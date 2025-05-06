#include <micromouse_cli/options/argument_parser.hpp>

#include <cassert>

void ArgumentParser::parse() {
  if (m_args.size() < 2)
    return;
  if (m_options.empty())
    return;

  for (size_t i = 1; i < m_args.size(); ++i) {
    if (m_args[i].empty())
      continue;

    Error error = parse_option(&i);

    const char* command = m_args[0].c_str();
    const char* option = m_args[i].c_str();

    switch (error) {
      case Error::NONE:
        break;
      case Error::INVALID_OPTION:
        (void)fprintf(stderr, "%s: invalid option: '%s'\n", command, option);
        break;
      case Error::MISSING_VALUE:
        (void)fprintf(stderr, "%s: missing argument for option: '%s'\n",
                      command, option);
        break;
      case Error::DUPLICATE_OPTION:
        (void)fprintf(stderr, "%s: duplicate option: '%s'\n", command, option);
        break;
    }

    if (error) {
      m_has_error = true;
      return;
    }
  }
}

ArgumentParser::Error ArgumentParser::parse_option(size_t* index) {
  assert(index != nullptr);
  assert(*index < m_args.size());

  std::string_view arg = m_args[*index];

  assert(!arg.empty());

  if (arg[0] != '-') {  // Not an option
    m_non_option_args.push_back(arg);
    return Error::NONE;
  }
  arg.remove_prefix(1);
  if (arg.empty())  // Empty option
    return Error::INVALID_OPTION;

  if (arg[0] == '-') {  // Second '-'
    arg.remove_prefix(1);
    if (arg.empty())
      return Error::INVALID_OPTION;
  }

  size_t matched_name_length = 0;
  const Option* option = &m_options[0];
  for (size_t i = 0; i < m_options.size(); ++i, ++option) {
    matched_name_length = match_option(*option, arg);
    if (matched_name_length > 0)
      break;
  }

  if (matched_name_length == 0)
    return Error::INVALID_OPTION;

  assert(matched_name_length <= arg.size());

  // No value required.
  if (!option->requires_value) {
    if (m_parsed_options.contains(option->id))
      return Error::DUPLICATE_OPTION;

    m_parsed_options.insert(option->id);
    return Error::NONE;
  }

  // Separated value.
  if (arg.length() == matched_name_length) {
    if (*index + 1 >= m_args.size())
      return Error::MISSING_VALUE;

    if (m_parsed_options.contains(option->id))
      return Error::DUPLICATE_OPTION;

    m_parsed_options.insert(option->id);
    m_parsed_option_values.emplace(option->id, m_args[++(*index)]);

    return Error::NONE;
  }

  // Connected value.
  arg.remove_prefix(matched_name_length);
  assert(!arg.empty());

  if (arg[0] == '=') {
    arg.remove_prefix(1);

    if (arg.empty())
      return Error::MISSING_VALUE;
  }

  if (m_parsed_options.contains(option->id))
    return Error::DUPLICATE_OPTION;

  m_parsed_options.insert(option->id);
  m_parsed_option_values.emplace(option->id, arg);
  return Error::NONE;
}

size_t ArgumentParser::match_option(const Option& option,
                                    std::string_view arg) const {
  assert(!arg.empty());

  const auto names = option.name.names();
  for (size_t i = 0; i < names.size(); ++i) {
    const char* name = names[i];
    if (name == nullptr)
      continue;

    bool matches = false;

    if (option.requires_value) {
      matches = arg.starts_with(name);
    } else {
      matches = (arg.compare(name) == 0);
    }
    if (matches)
      return strlen(name);
  }

  return 0;
}
