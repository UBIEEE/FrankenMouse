#pragma once

#include <micromouse_cli/robot/song.hpp>
#include <micromouse_cli/communication/communication_manager.hpp>
#include <micromouse_cli/commands/command.hpp>
#include <micromouse_cli/diagnostics.hpp>
#include <micromouse_cli/options/argument_parser.hpp>
#include <cstdio>
#include <map>
#include <string>

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

  static constexpr const char* s_stop = "Stop";
  static constexpr const char* s_home_depot_theme = "HomeDepotTheme";
  static constexpr const char* s_nokia_ringtone = "NokiaRingtone";

  static inline const std::map<std::string, RobotSong> s_songs{
      {s_stop, RobotSong::NONE},
      {s_home_depot_theme, RobotSong::HOME_DEPOT},
      {s_nokia_ringtone, RobotSong::NOKIA},
  };

  static inline const std::vector<std::string> s_non_options{
      s_stop,
      s_home_depot_theme,
      s_nokia_ringtone,
  };

 public:
  static const char* name() { return "song-play"; }
  static PromptInfo prompt_info() {
    return PromptInfo{.usage_text = "song-play <song> [options]",
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
  CommunicationManager& m_communication_manager;

  bool m_is_done = true;
  bool m_keep_alive = false;
  RobotSong m_song = RobotSong::NONE;

 public:
  SongPlayCommand(const CommandArguments args, CommunicationManager& communication_manager);
  ~SongPlayCommand();

  void process() override;

  CommandStatus status() const override { return m_is_done ? CommandStatus::DONE : CommandStatus::CONTINUING; }

 private:
  // Returns true if the command should keep running.
  bool validate_args();
};
