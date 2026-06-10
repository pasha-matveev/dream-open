#include "strategy/vigilans.h"

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/keeper.h"

void VigilansController::observe(const std::optional<Vec>& opt_pos,
                                 long long now) {
  const auto& cfg = *config->strategy->keeper->vigilans;
  if (!opt_pos) {
    // Мяч не свежий — начинаем отсчёт покоя заново.
    history_.clear();
    return;
  }
  if (history_.empty()) continuous_since_ = now;
  history_.emplace_back(now, *opt_pos);
  // Держим только последние still_ms; фронт ровно на границе окна оставляем.
  while (!history_.empty() && now - history_.front().first > cfg.still_ms) {
    history_.pop_front();
  }
}

bool VigilansController::stationary(long long now) const {
  const auto& cfg = *config->strategy->keeper->vigilans;
  if (history_.empty()) return false;
  // Окно ещё не набрано целиком.
  if (now - continuous_since_ < cfg.still_ms) return false;

  // Центроид окна (опорная точка устойчивее к одиночному выбросу, чем
  // последняя позиция — фильтра-сглаживателя нет).
  Vec sum{0, 0};
  for (const auto& s : history_) sum += s.second;
  Vec centroid = sum * (1.0 / static_cast<double>(history_.size()));

  // Допускаем долю выбросов (сырые позиции без фильтра): достаточно, чтобы
  // не менее still_inlier_frac сэмплов лежали в пределах радиуса от центроида.
  size_t inliers = 0;
  for (const auto& s : history_) {
    if ((s.second - centroid).len() <= cfg.still_radius) inliers++;
  }
  return inliers >= cfg.still_inlier_frac * static_cast<double>(history_.size());
}

bool VigilansController::cooldown_passed(long long now) const {
  const auto& cfg = *config->strategy->keeper->vigilans;
  return now - last_deactivated_at_ >= cfg.cooldown_ms;
}

void VigilansController::activate(long long now) {
  active_ = true;
  activated_at_ = now;
  // Сброс состояния фронта emitter: новый цикл начинаем без «удерживали мяч»,
  // иначе застрявший had_emitter_ (напр. после таймаута-с-мячом) даст ложный
  // удар на первом же тике следующей активации.
  had_emitter_ = false;
}

void VigilansController::deactivate(long long now) {
  active_ = false;
  last_deactivated_at_ = now;
}
