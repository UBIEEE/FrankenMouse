#pragma once

#include <cstdint>
#include <micromouse/audio/note.hpp>
#include <micromouse/audio/song.hpp>
#include <micromouse/hardware/buzzer.hpp>
#include <micromouse/hardware/feedback.hpp>
#include <micromouse/subsystem.hpp>
#include <span>

namespace audio {

class AudioPlayer : public Subsystem {
  hardware::Feedback& m_feedback = get_platform_feedback();
  hardware::Buzzer& m_buzzer = get_platform_buzzer();

 private:
  struct SongHandle {
    std::span<const Note> notes;
    uint16_t ticks_per_note;
    bool pause_between_notes = true;
  };

 private:
  static const SongHandle m_songs[uint8_t(Song::_COUNT)];  // songs.cpp

  const SongHandle* m_song_handle = nullptr;
  bool m_song_repeat = false;
  uint16_t m_note_index = 0;
  uint16_t m_note_ticks = 0;

  bool m_should_stop = false;
  uint8_t m_is_playing = false;

 public:
  AudioPlayer();

  void periodic() override;
  void publish_extra_feedback() override;

  /**
   * @brief Immediately starts playing a song from the beginning.
   *
   * @param song Which song to play.
   * @param repeat Whether to repeat the song when finished.
   */
  void play_song(Song song, bool repeat = false);

  void quiet() {
    m_song_repeat = false;
    m_should_stop = true;
  }

  bool is_playing() const { return (m_song_handle != nullptr); }

 private:
  void end_song();
};

}  // namespace audio
