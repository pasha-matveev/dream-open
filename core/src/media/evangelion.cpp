#include "media/music.h"

// "Zankoku na Tenshi no Theze" — Neon Genesis Evangelion OP.
// Транскрипция из MIDI (bitmidi.com/evangelion-cruel-angels-thesis-mid),
// channel 6 — знаменитая струнная остинато ascending arpeggio
// (Bb→C→D→Eb→F→Ab→B→Bb), которая идёт через весь верс и узнаётся с
// первых секунд трека. Top-voice extraction, 32 ноты ≈ 1 цикл, дальше
// play_song зацикливает.
std::vector<Note> notes_evangelion = {
    {bb, 4, 0.5},     {cnat, 5, 0.5},   {dnat, 5, 0.5}, {eb, 5, 0.5},
    {fnat, 5, 0.5},   {gsharp, 5, 0.5}, {bnat, 5, 0.5}, {bb, 5, 1.0},

    {fsharp, 5, 3.0}, {anat, 5, 0.5},   {cnat, 6, 0.5}, {bb, 5, 0.5},
    {gsharp, 5, 0.5}, {fsharp, 5, 0.5}, {fnat, 5, 1.0}, {dnat, 5, 3.0},

    {0, 0, 0.25},     {fnat, 5, 0.5},   {gnat, 5, 0.5}, {fnat, 5, 0.5},
    {eb, 5, 0.5},     {dnat, 5, 0.5},   {eb, 5, 1.0},   {cnat, 5, 3.0},

    {eb, 5, 0.5},     {fnat, 5, 0.5},   {eb, 5, 0.5},   {dnat, 5, 0.5},
    {cnat, 5, 0.5},   {dnat, 5, 3.0},

    {0, 0, 1.0},
};
