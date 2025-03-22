#include <micromouse/audio/audio_player.hpp>

using namespace audio;

AudioPlayer::AudioPlayer() {
  play_song(Song::STARTUP);
}

void AudioPlayer::periodic() {
  if (!m_song_handle)
    return;

  if (m_should_stop) {
    end_song();
    m_should_stop = false;
    return;
  }

  if (m_note_index == 0 && m_note_ticks == 0) {
    m_is_playing = true;
    m_feedback.publish_topic(FeedbackTopicSend::MUSIC_IS_PLAYING, &m_is_playing);
  }

  if (m_note_ticks++ == 0) {
    const Note note = m_song_handle->notes[m_note_index];
    m_buzzer.play_note(note);
  }

  // End of a note.
  if (m_note_ticks == m_song_handle->ticks_per_note) {
    m_note_ticks = 0;

    if (m_song_handle->pause_between_notes) {
      // Add a small rest to separate notes.
      m_buzzer.play_note(REST);
    }

    // Check if this is the last note in the song.
    if (++m_note_index == m_song_handle->notes.size()) {
      end_song();
    }
  }
}

void AudioPlayer::publish_extra_feedback() {
  m_feedback.publish_topic(FeedbackTopicSend::MUSIC_IS_PLAYING, &m_is_playing);
}

void AudioPlayer::play_song(Song song, bool repeat) {
  if (song >= Song::_COUNT)
    return;

  m_song_handle = &m_songs[uint8_t(song)];
  m_song_repeat = repeat;
  m_note_index = 0;
  m_note_ticks = 0;
  m_should_stop = false;
}

void AudioPlayer::end_song() {
  m_buzzer.play_note(REST);

  if (m_song_repeat) {
    m_note_index = 0;
    m_note_ticks = 0;
    m_should_stop = false;
    return;
  }

  if (m_song_handle != nullptr) {
    m_is_playing = false;
    m_song_handle = nullptr;
    m_feedback.publish_topic(FeedbackTopicSend::MUSIC_IS_PLAYING, &m_is_playing);
  }
}
