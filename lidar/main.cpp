// ./main --channel --serial /dev/cu.usbserial-110 460800

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
static inline void delay(sl_word_size_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  };
  if (ms != 0) usleep(ms * 1000);
}

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

using namespace sl;
using namespace std;

std::atomic<bool> keepRunning(true);

// Signal handler for SIGTERM
void handle_sigterm(int signum) {
  cout << "\nCaught SIGTERM (signal " << signum
       << "), preparing to exit gracefully.\n"
       << flush;
  keepRunning.store(false);
}

bool checkSLAMTECLIDARHealth(ILidarDriver* drv) {
  sl_result op_result;
  sl_lidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  if (SL_IS_OK(op_result)) {
    cout << "SLAMTEC Lidar health status : " << healthinfo.status << "\n\n"
         << flush;
    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
      cout << "Error, slamtec lidar internal error detected. Please "
              "reboot the device to retry.\n"
           << flush;
      return false;
    } else {
      return true;
    }
  } else {
    cout << "Error, cannot retrieve the lidar health code: " << op_result
         << "\n"
         << flush;
    return false;
  }
}

bool ctrl_c_pressed;
void ctrlc(int) { ctrl_c_pressed = true; }

// 2D point struct
struct Point {
  double x, y;
};

// Struct to hold the resulting minimal bounding rectangle
struct Rect {
  // The four corners of the rectangle in rotation order
  vector<Point> corners;
  double width;   // The rectangle width
  double height;  // The rectangle height
  double angle;   // Rotation angle in radians
};

// Cross product of OA and OB vectors, i.e. z-component of (OA x OB).
static double cross(const Point& O, const Point& A, const Point& B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Computes the convex hull of a set of points using Andrew's monotone chain
// algorithm. Returns the convex hull as a vector of points in counter-clockwise
// order.
static vector<Point> computeConvexHull(vector<Point> pts) {
  // Sort the points primarily by x, then by y
  sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
    if (fabs(a.x - b.x) < 1e-12) return a.y < b.y;
    return a.x < b.x;
  });

  // Build lower hull
  vector<Point> hull;
  for (int i = 0; i < (int)pts.size(); i++) {
    while (hull.size() >= 2 &&
           cross(hull[hull.size() - 2], hull[hull.size() - 1], pts[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(pts[i]);
  }

  // Build upper hull
  for (int i = (int)pts.size() - 2, t = (int)hull.size() + 1; i >= 0; i--) {
    while ((int)hull.size() >= t &&
           cross(hull[hull.size() - 2], hull[hull.size() - 1], pts[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(pts[i]);
  }

  hull.pop_back();  // remove the duplicate last point
  return hull;
}

// Compute the average of the rectangle's four corners, i.e. its center.
static Point computeRectCenter(const Rect& rect) {
  double cx = 0.0, cy = 0.0;
  for (auto& corner : rect.corners) {
    cx += corner.x;
    cy += corner.y;
  }
  // average over the 4 corners
  cx /= 4.0;
  cy /= 4.0;
  return {cx, cy};
}

// Rotate a point (px, py) by 'theta' around origin (0, 0)
static Point rotatePoint(double px, double py, double theta) {
  double cosT = cos(theta);
  double sinT = sin(theta);
  Point result;
  result.x = px * cosT - py * sinT;
  result.y = px * sinT + py * cosT;
  return result;
}

// Given a set of 2D points, returns the minimal-area bounding rectangle
// corners, width, height, and rotation angle (in radians).
Rect minimalAreaBoundingRectangle(const vector<Point>& points) {
  // Special case: if no points, return empty result
  Rect bestRect;
  bestRect.width = 0;
  bestRect.height = 0;
  bestRect.angle = 0;

  if (points.size() < 1) {
    return bestRect;  // no rectangle
  }

  // Compute convex hull
  vector<Point> hull = computeConvexHull(points);
  int n = (int)hull.size();
  if (n < 2) {
    // All points are the same or there's only one point.
    bestRect.corners = hull;
    return bestRect;
  }

  // indices for calipers
  int k = 1, l = 1, m = 1;

  // area, angle, bounding rectangle corners
  double minArea = numeric_limits<double>::infinity();
  double bestAngle = 0.0;
  // We'll store corners in (0, 0) for now
  vector<Point> bestCorners(4);

  // For each edge i of the hull
  for (int i = 0; i < n; i++) {
    // next index
    int i2 = (i + 1) % n;

    // Edge from hull[i] to hull[i2]
    // Angle of this edge
    double dx = hull[i2].x - hull[i].x;
    double dy = hull[i2].y - hull[i].y;
    double edgeAngle = atan2(dy, dx);

    // Move k while area side is increasing
    // We'll do a rotating calipers approach.
    // But we primarily only need to track the opposite side, leftmost,
    // rightmost etc.

    // Let's find the leftmost, rightmost, topmost points relative to this
    // edge. We'll keep track using the existing indices k, l, m.

    // Move k while cross(...) is positive
    while (fabs(cross(hull[i], hull[i2], hull[(k + 1) % n])) >
           fabs(cross(hull[i], hull[i2], hull[k]))) {
      k = (k + 1) % n;
    }
    // Move l while the dot(...) is increasing for leftmost
    // Move m while the dot(...) is increasing for rightmost
    // We can approximate with the standard rotating calipers method.

    // But let's do a simpler bounding box approach:
    // For each point in hull, rotate it so that the edge is aligned with
    // x-axis, then track min_x, max_x, min_y, max_y.

    double cosA = cos(-edgeAngle);
    double sinA = sin(-edgeAngle);

    double minX = numeric_limits<double>::infinity();
    double maxX = -numeric_limits<double>::infinity();
    double minY = numeric_limits<double>::infinity();
    double maxY = -numeric_limits<double>::infinity();

    // Rotate all hull points, find bounding box in that coordinate frame
    for (int h = 0; h < n; h++) {
      // rotate hull[h] by -edgeAngle
      double rx = hull[h].x * cosA - hull[h].y * sinA;
      double ry = hull[h].x * sinA + hull[h].y * cosA;
      if (rx < minX) minX = rx;
      if (rx > maxX) maxX = rx;
      if (ry < minY) minY = ry;
      if (ry > maxY) maxY = ry;
    }

    double width = maxX - minX;
    double height = maxY - minY;
    double area = width * height;

    if (area < minArea) {
      minArea = area;
      bestAngle = edgeAngle;

      // corners in the rotated frame
      // bottom-left
      Point bl{minX, minY};
      // bottom-right
      Point br{maxX, minY};
      // top-right
      Point tr{maxX, maxY};
      // top-left
      Point tl{minX, maxY};

      // rotate them back
      auto rotateBack = [&](const Point& p) {
        double xx = p.x * cos(edgeAngle) - p.y * sin(edgeAngle);
        double yy = p.x * sin(edgeAngle) + p.y * cos(edgeAngle);
        return Point{xx, yy};
      };

      bestCorners[0] = rotateBack(bl);
      bestCorners[1] = rotateBack(br);
      bestCorners[2] = rotateBack(tr);
      bestCorners[3] = rotateBack(tl);

      bestRect.width = width;
      bestRect.height = height;
    }
  }

  bestRect.angle = bestAngle;
  bestRect.corners = bestCorners;
  return bestRect;
}

vector<Point> generate_points(int n) {
  vector<Point> v;

  for (int i = 0; i < n; i++)
    v.push_back(Point{100.f + (rand() % 301), 100.f + (rand() % 301)});

  for (int i = 0; i < 2; i++) {
    Point cluster_center =
        Point{100.f + (rand() % 301), 100.f + (rand() % 301)};
    v.push_back(cluster_center);

    for (int i = 0; i < 20; i++)
      v.push_back(Point{cluster_center.x - 25.f + (rand() % 51),
                        cluster_center.y - 25.f + (rand() % 51)});
  }

  return v;
}

// Pads a rectangle by `padding` units on all sides, preserving its angle.
Rect padRectangle(const Rect& rect, double padding) {
  // 1. Compute the center of the existing rectangle.
  double cx = 0.0, cy = 0.0;
  for (auto& corner : rect.corners) {
    cx += corner.x;
    cy += corner.y;
  }
  // Each rectangle has 4 corners, so the average is the center.
  cx /= 4.0;
  cy /= 4.0;

  // 2. Compute the new width and height with uniform padding on both sides
  double newWidth = rect.width + 2.0 * padding;
  double newHeight = rect.height + 2.0 * padding;

  // 3. Build corners in an unrotated coordinate system, centered at (0, 0).
  //    We'll define half-width and half-height in that local space.
  double halfW = newWidth / 2.0;
  double halfH = newHeight / 2.0;

  // Bottom-left, bottom-right, top-right, top-left (in local space).
  vector<Point> localCorners = {
      {-halfW, -halfH}, {halfW, -halfH}, {halfW, halfH}, {-halfW, halfH}};

  // 4. Rotate and translate each corner to the global (original) coordinate
  // system.
  //    The final rectangle keeps the original rotation and center.
  vector<Point> globalCorners(4);
  for (int i = 0; i < 4; i++) {
    Point rotated =
        rotatePoint(localCorners[i].x, localCorners[i].y, rect.angle);
    globalCorners[i] = {rotated.x + cx, rotated.y + cy};
  }

  // 5. Create and return the new rectangle.
  Rect paddedRect;
  paddedRect.corners = globalCorners;
  paddedRect.width = newWidth;
  paddedRect.height = newHeight;
  paddedRect.angle = rect.angle;  // unchanged
  return paddedRect;
}

// This function partitions 'points' into two sets: 'inside' and 'outside'
// relative to the given rotated rectangle 'rect'.
void partitionPointsInsideOutside(const vector<Point>& points, const Rect& rect,
                                  vector<Point>& inside,
                                  vector<Point>& outside) {
  // 1) Compute the rectangle's center.
  //    We'll rotate points about this center so that the rectangle
  //    aligns with the axes.
  Point center = computeRectCenter(rect);

  // 2) For convenience, half-width and half-height
  double halfW = rect.width / 2.0;
  double halfH = rect.height / 2.0;

  // 3) For each point, translate it so center is at origin, then rotate by
  // -angle
  double negativeAngle = -rect.angle;
  for (const auto& p : points) {
    // translate
    double tx = p.x - center.x;
    double ty = p.y - center.y;
    // rotate
    Point rotated = rotatePoint(tx, ty, negativeAngle);

    // 4) Check if the rotated point is within the bounding box
    if ((rotated.x >= -halfW && rotated.x <= halfW) &&
        (rotated.y >= -halfH && rotated.y <= halfH)) {
      inside.push_back(p);
    } else {
      outside.push_back(p);
    }
  }
}

// Euclidean distance helper
static double distance2D(const Point& a, const Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

static double angle2D(const Point& a, const Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return atan2(dy, dx);
}

/*
  clusterPoints:
    - points: the input set of points
    - distThreshold: maximum distance to consider two points as neighbors
  Returns:
    A vector of clusters, where each cluster is itself a vector of Points.
*/
vector<vector<Point>> clusterPoints(const vector<Point>& points,
                                    double distThreshold) {
  vector<vector<Point>> clusters;
  if (points.empty()) return clusters;

  int n = (int)points.size();
  vector<bool> visited(n, false);

  // For each unvisited point, perform a BFS (or DFS) to find all connected
  // neighbors
  for (int i = 0; i < n; i++) {
    if (!visited[i]) {
      visited[i] = true;
      // Start a new cluster
      vector<Point> cluster;
      queue<int> q;
      q.push(i);

      // BFS: gather all points within distThreshold
      while (!q.empty()) {
        int curIdx = q.front();
        q.pop();
        cluster.push_back(points[curIdx]);

        // Check all unvisited points to see if they belong to this
        // cluster
        for (int j = 0; j < n; j++) {
          if (!visited[j]) {
            if (distance2D(points[curIdx], points[j]) < distThreshold) {
              visited[j] = true;
              q.push(j);
            }
          }
        }
      }
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

void draw_dot(sf::RenderWindow& window, Point pos, sf::Color color,
              float radius = 1.f) {
  sf::CircleShape dot(radius);
  dot.setFillColor(color);
  dot.move(sf::Vector2f(pos.x - radius, pos.y - radius));
  window.draw(dot);
}

void draw_dots(sf::RenderWindow& window, vector<Point> points, sf::Color color,
               float radius = 1.f) {
  for (Point pos : points) draw_dot(window, pos, color, radius);
}

void draw_convex(sf::RenderWindow& window, vector<Point> points,
                 sf::Color outline_color,
                 sf::Color fill_color = sf::Color::Transparent,
                 float thickness = 1.f) {
  sf::ConvexShape convex;
  convex.setPointCount(points.size());
  for (int i = 0; i < points.size(); i++)
    convex.setPoint(i, sf::Vector2f(points[i].x, points[i].y));
  convex.setFillColor(fill_color);
  convex.setOutlineColor(outline_color);
  convex.setOutlineThickness(thickness);
  window.draw(convex);
}

void draw_line(sf::RenderWindow& window, const Point& p1, const Point& p2,
               const sf::Color& color, float thickness = 1.f) {
  sf::RectangleShape line;
  line.setSize(sf::Vector2f(distance2D(p1, p2), thickness));
  line.setFillColor(color);
  line.setPosition(sf::Vector2f(p1.x, p1.y));
  line.setRotation(sf::Angle(sf::radians(angle2D(p2, p1))));
  window.draw(line);
}

void draw_circle(sf::RenderWindow& window, Point pos, float radius,
                 sf::Color outline_color,
                 sf::Color fill_color = sf::Color::Transparent,
                 float thickness = 1.f) {
  sf::CircleShape circle(radius);
  circle.setPosition(sf::Vector2f(pos.x - radius, pos.y - radius));
  circle.setOutlineThickness(thickness);
  circle.setOutlineColor(outline_color);
  circle.setFillColor(fill_color);
  window.draw(circle);
}

int main(int argc, const char* argv[]) {
  const char* port;
  int baudrate = 460800;
  bool preview;
  bool running = true;
  sl_result op_result;
  IChannel* _channel;
  ILidarDriver* drv = *createLidarDriver();
  vector<LidarScanMode> scanModes;
  LidarMotorInfo motorInfo;

  if (argc > 1) {
    preview = atoi(argv[1]);
    port = argv[2];
  } else {
    preview = 0;
    // port = "/dev/cu.usbserial-110";
    port = "/dev/ttyUSB1";
  }

  Point center = {400, 400};
  sf::RenderWindow* window = nullptr;
  if (preview) {
    window = new sf::RenderWindow(sf::VideoMode({800, 800}), "Lidar Preview");
  }

  if (!drv) {
    cout << "insufficent memory, exit\n" << flush;
    exit(-2);
  }

  bool connectSuccess = false;

  auto try_connect = [&](int number) {
    sl_lidar_response_device_info_t devinfo;

    string attempt_port = "/dev/ttyUSB" + std::to_string(number);

    // Connect to the lidar
    _channel = (*createSerialPortChannel(attempt_port, baudrate));
    if (SL_IS_OK((drv)->connect(_channel))) {
      op_result = drv->getDeviceInfo(devinfo);

      if (SL_IS_OK(op_result)) {
        connectSuccess = true;
      } else {
        delete drv;
        drv = NULL;
      }
    }

    // Failed to connect
    if (!connectSuccess) {
      return false;

    } else {
      return true;
    }
  };

  for (int i = 1; i < 10 && !connectSuccess; ++i) {
    try_connect(i);
  }

  if (!connectSuccess) {
    cout << "Error, cannot bind to the specified serial port " << port << ".\n"
         << flush;
    goto on_finished;
  }

  // Additional prints

  // Lidar health
  // if (!checkSLAMTECLIDARHealth(drv))
  // {
  //     goto on_finished;
  // }

  // Supported scan modes
  // drv->getAllSupportedScanModes(scanModes);
  // for (auto &scanMode : scanModes)
  // {
  //     // print out all info of each scan mode
  //     printf("Scan mode: %s\n", scanMode.scan_mode);
  //     printf("ID: %d\n", scanMode.id);
  //     printf("US per sample: %f\n", scanMode.us_per_sample);
  //     printf("Max distance: %f\n", scanMode.max_distance);
  //     printf("Ans type: %d\n\n", scanMode.ans_type);
  // }

  // Motor info
  // drv->getMotorInfo(motorInfo);
  // printf("Motor min speed: %d\n", motorInfo.min_speed);
  // printf("Motor max speed: %d\n", motorInfo.max_speed);
  // printf("Motor desired speed: %d\n\n", motorInfo.desired_speed);

  // Register the SIGTERM handler
  struct sigaction action;
  action.sa_handler = handle_sigterm;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;

  if (sigaction(SIGTERM, &action, nullptr) < 0) {
    cout << "Error setting up SIGTERM handler.\n" << flush;
    return 1;
  }

  // cout << "C++ program started. PID: " << getpid() << "\n" << flush;
  // cout << "Waiting for SIGTERM.\n" << flush;

  signal(SIGINT, ctrlc);

  drv->setMotorSpeed();
  drv->startScan(0, 1);

  while (!ctrl_c_pressed && keepRunning.load() && running) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    vector<Point> points;
    vector<Rect> result_data;

    op_result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(op_result)) {
      drv->ascendScanData(nodes, count);
      for (int pos = 0; pos < (int)count; ++pos) {
        float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
        float dist = nodes[pos].dist_mm_q2 / 4.0f;
        int quality =
            nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (quality && dist < 3000.f) {
          double x = center.x + dist / 10.f * sin(angle * M_PI / 180.f);
          double y = center.y - dist / 10.f * cos(angle * M_PI / 180.f);
          points.push_back(Point{x, y});
        }
        // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
        // (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "
        // ", angle, dist, quality);
      }
      // float frequency;
      // drv->getFrequency(scanModes[0], nodes, count, frequency);
      // printf("Frequency: %f", frequency);
    }

    Rect field = minimalAreaBoundingRectangle(points);
    result_data.push_back(field);
    Rect inner = padRectangle(field, -20);
    vector<Point> inside, outside;
    partitionPointsInsideOutside(points, inner, inside, outside);
    vector<vector<Point>> clusters = clusterPoints(inside, 10);
    for (auto& cluster : clusters)
      if (cluster.size() > 5)
        result_data.push_back(minimalAreaBoundingRectangle(cluster));

    if (preview) {
      running = window->isOpen();
      while (const optional event = window->pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
          window->close();
        }
      }
      window->clear(sf::Color::Black);
      draw_circle(*window, center, 300, sf::Color::White);
      draw_dot(*window, center, sf::Color::Green, 2);
      for (int i = 0; i < result_data.size(); i++)
        draw_convex(*window, result_data[i].corners,
                    i ? sf::Color::Red : sf::Color::Green);
      draw_convex(*window, inner.corners, sf::Color::White);
      draw_dots(*window, outside, sf::Color::Green);
      draw_dots(*window, inside, sf::Color::White);
      window->display();
    }

    cout << "Data ";
    for (Rect& object : result_data) {
      Point object_center = computeRectCenter(object);
      double angle =
          atan2(center.y - object_center.y, object_center.x - center.x);
      double dist = sqrt(pow(center.y - object_center.y, 2) +
                         pow(center.x - object_center.x, 2));
      double rotation = object.angle;
      double width = object.width;
      double height = object.height;
      cout << setprecision(5) << angle << ' ';
      cout << setprecision(5) << dist << ' ';
      cout << setprecision(5) << rotation << ' ';
      cout << (int)width << ' ';
      cout << (int)height << ' ';
      // printf(" %.2f %.2f %.2f %.2f %.2f", angle, dist, rotation, width,
      // height);
    }
    // printf("\n");
    cout << '\n' << flush;
  }

  drv->stop();
  delay(200);
  drv->setMotorSpeed(0);

  cout << "Exiting.\n" << flush;

on_finished:
  if (drv) {
    delete drv;
    drv = NULL;
  }
  delete window;
  return 0;
}
