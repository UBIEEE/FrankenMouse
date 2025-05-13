#pragma once

#include <cstdio>
#include <map>
#include <micromouse_cli/audio/song.hpp>
#include <micromouse_cli/ble_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>

class SongPlayCommand final : public Command {
  enum {
    OPTION_HELP,
    OPTION_KEEP_ALIVE,
  };

  static inline const std::vector<Option> s_options{
      // clang-format off
      {OPTION_HELP,       OptionName("help",       "h"), false, nullptr, "Show this help message"},
      {OPTION_KEEP_ALIVE, OptionName("keep-alive", "k"), false, nullptr, "Keep the process alive until the song is done playing"},
      // clang-format on
  };

  static constexpr const char* s_home_depot_theme = "HomeDepotTheme";
  static constexpr const char* s_nokia_ringtone = "NokiaRingtone";

  static inline const std::map<std::string, Song> s_songs{
      {s_home_depot_theme, Song::HOME_DEPOT},
      {s_nokia_ringtone, Song::NOKIA},
  };

  static inline const std::vector<std::string> s_non_options{
      s_home_depot_theme,
      s_nokia_ringtone,
  };

 public:
  COMMAND_NAME("song-play")

  static PromptInfo prompt_info() {
    return PromptInfo{
        .usage_text = "song-play [song] [options]",
        .short_description_text = "Play a song on the MicroMouse",
        .long_description_text =
            "Play a song on the MicroMouse. By default, this command will "
            "start playing a song and then exit. Use the --keep-alive option "
            "to keep the process alive until the song is done playing. The "
            "song will be stopped if the command is interrupted.",
        .options = s_options,
        .non_options_title = "Songs",
        .non_options = s_non_options};
  }

 private:
  ArgumentParser m_arg_parser;
  BLEManager& m_ble_manager;

  bool m_is_done = false;
  bool m_keep_alive = false;
  Song m_song = Song::NONE;

 public:
  SongPlayCommand(const CommandArguments args, BLEManager& ble_manager);
  ~SongPlayCommand();

  bool is_done() const override { return m_is_done; }

  CommandProcessResult process() override;

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
