#include <micromouse_cli/commands/song_play.hpp>

SongPlayCommand::SongPlayCommand(const CommandArguments args,
                                 BLEManager& ble_manager)
    : Command(args), m_arg_parser(args, s_options), m_ble_manager(ble_manager) {
  if (!validate_args() && m_song != Song::NONE)
    return;

  m_ble_manager.write<BLETopicWrite::MUSIC_PLAY_SONG>(m_song);
}

SongPlayCommand::~SongPlayCommand() {
  if (!m_keep_alive)
    return;

  m_ble_manager.write<BLETopicWrite::MUSIC_PLAY_SONG>(Song::NONE);
}

CommandProcessResult SongPlayCommand::process() {
  if (m_ble_manager.get_data<BLETopicNotify::MUSIC_IS_PLAYING>()) {
    return CommandProcessResult::CONTINUE;
  }
  return CommandProcessResult::DONE;
}

bool SongPlayCommand::validate_args() {
  if (m_arg_parser.has_error())
    return false;

  const std::unordered_set<int>& options = m_arg_parser.parsed_options();
  std::span<const std::string_view> non_option_args =
      m_arg_parser.non_option_args();

  // Help

  if (options.contains(OPTION_HELP)) {
    help(name(), prompt_info(), stdout);
    return false;
  }

  // Port

  if (non_option_args.empty()) {
    report_error(name(), "missing song argument");
    return false;
  } else if (non_option_args.size() > 1) {
    report_error(name(), "too many arguments");
    return false;
  }

  std::string song_name(non_option_args[0]);

  auto it = s_songs.find(song_name);
  if (it == s_songs.end()) {
    report_error(name(), "unknown song: %s", non_option_args[0].data());
    return false;
  }

  m_song = it->second;

  // Keep alive

  m_keep_alive = options.contains(OPTION_KEEP_ALIVE);
  m_is_done = (m_keep_alive == false);

  return true;
}
