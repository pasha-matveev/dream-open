#pragma once

// Режим торможения ребра поля (см. Field::brake_modes_).
//   Normal — обычное торможение об ребро с коэффициентом decel_k.
//   Off    — Segment::apply на ребре не вызывается (виртуальная граница без
//            физической стенки).
//   Low    — ослабленное торможение: вместо decel_k используется decel_k_low.
//            Влияет только на физическое торможение (Branch 1, d>=0);
//            push-out (d<0) остаётся обычным.
enum class BrakeMode { Normal, Off, Low };
