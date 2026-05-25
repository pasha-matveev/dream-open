#include "media/music.h"

// "Zankoku na Tenshi no Theze" — Neon Genesis Evangelion OP.
// Transcribed from Pandarc piano arrangement
// (pandarcpiano.com/a-cruel-angels-thesis-free-piano-sheet-music/):
// E♭ major, 4/4, ♩=126. Только верхняя мелодическая линия, без аккомпанемента.
std::vector<Note> notes_evangelion = {
    // Верс: "Zan-ko-ku na ten-shi no / you ni / shou-nen yo"
    {gnat, 5, 0.5},   {gnat, 5, 0.5}, {gnat, 5, 0.5},   {gnat, 5, 0.5},
    {bb, 5, 0.5},     {gsharp, 5, 0.5}, {gnat, 5, 1.0},
    {0, 0, 0.25},
    {fnat, 5, 0.5},   {gnat, 5, 0.5}, {fnat, 5, 0.5},   {eb, 5, 1.5},
    {0, 0, 0.5},

    // "shin-wa ni na-re"
    {gnat, 5, 0.5},   {gsharp, 5, 0.5}, {bb, 5, 0.5},   {gsharp, 5, 0.5},
    {gnat, 5, 0.5},   {fnat, 5, 0.5},   {eb, 5, 2.0},
    {0, 0, 0.5},

    // Пред-припев: "Ao-i ka-ze ga i-ma"
    {gnat, 5, 0.5},   {gsharp, 5, 0.5}, {bb, 5, 0.5},   {cnat, 6, 0.5},
    {bb, 5, 0.5},     {gsharp, 5, 0.5}, {gnat, 5, 1.5},
    {0, 0, 0.5},

    // "Mu-ne no do-a wo ta-ta-i-te mo"
    {gnat, 5, 0.5},   {fnat, 5, 0.5},   {eb, 5, 0.5},   {fnat, 5, 0.5},
    {gnat, 5, 1.0},   {fnat, 5, 0.5},   {eb, 5, 0.5},
    {dnat, 5, 2.0},
    {0, 0, 0.5},

    // Припев — главный хук: "Zan-ko-ku na ten-shi no te-e-ze"
    {eb, 6, 0.5},     {dnat, 6, 0.5},   {cnat, 6, 0.5}, {dnat, 6, 0.5},
    {eb, 6, 1.0},     {cnat, 6, 0.5},   {bb, 5, 0.5},
    {gsharp, 5, 0.5}, {gnat, 5, 0.5},   {gsharp, 5, 0.5}, {bb, 5, 0.5},
    {gsharp, 5, 0.5}, {gnat, 5, 0.5},   {fnat, 5, 0.5}, {eb, 5, 0.5},
    {eb, 5, 1.5},
    {0, 0, 0.5},

    // "Ma-do-be ka-ra ya-ga-te to-bi-ta-tsu"
    {bb, 5, 0.5},     {cnat, 6, 0.5},   {dnat, 6, 0.5}, {eb, 6, 0.5},
    {dnat, 6, 0.5},   {cnat, 6, 0.5},   {bb, 5, 1.0},
    {gsharp, 5, 0.5}, {gnat, 5, 0.5},   {fnat, 5, 0.5}, {gnat, 5, 0.5},
    {fnat, 5, 0.5},   {eb, 5, 1.5},
    {0, 0, 1.0},

    // Хвост-связка перед циклом
    {gnat, 5, 0.5},   {gnat, 5, 0.5},   {bb, 5, 0.5},   {gnat, 5, 0.5},
    {eb, 5, 2.0},
    {0, 0, 2.0},
};
