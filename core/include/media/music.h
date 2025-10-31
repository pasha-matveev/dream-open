#pragma once

#include <vector>

constexpr int beat = 500;
constexpr float cnat = 16.35;
constexpr float csharp = 17.32;
constexpr float dnat = 18.35;
constexpr float eb = 19.45;
constexpr float enat = 20.60;
constexpr float fnat = 21.83;
constexpr float fsharp = 23.12;
constexpr float gnat = 24.50;
constexpr float gsharp = 25.96;
constexpr float anat = 27.50;
constexpr float bb = 29.14;
constexpr float bnat = 30.87;

struct Note {
    double base;
    int octave;
    double length;
};

extern std::vector<Note> notes1;

const std::vector<Note> &get_notes(int);