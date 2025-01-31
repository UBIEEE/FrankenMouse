#include "Navigation/navigator.hpp"

#include "Basic/constants.hpp"
#include "custom_stm.h"

void Navigator::process() {
  if (m_done) return;

  if (!m_drive.is_done()) return;

  switch (m_mode) {
    using enum Mode;
  case SEARCH:
  case SOLVE_SLOW:
    next_motion_slow();
    break;
  case SOLVE_FAST:
    next_motion_fast();
    break;
  }
}

void Navigator::next_motion_slow() {
  if (m_mode == Mode::SEARCH) {
    m_solver->solve(m_target);
  }

  // TODO: Determine next motion...

  /*
  if (finished) {
    m_done = true;
  }
  */
}

void Navigator::next_motion_fast() {

  /*
  if (finished) {
    m_done = true;
  }
  */
}

void Navigator::reset_position(maze::Coordinate position,
                               maze::Direction direction,
                               float cell_position_mm) {
  m_position               = position;
  m_direction              = direction;
  m_start_cell_position_mm = cell_position_mm;
}

void Navigator::search_to(maze::CoordinateSpan target, const Solver& solver) {
  m_done   = false;
  m_target = target;
  m_mode   = Mode::SEARCH;
  m_solver = &solver;

  m_drive.set_speeds(m_speeds.m_normal_speeds);
}

void Navigator::solve_to(maze::CoordinateSpan target, bool fast) {
  m_done = false;
  m_target = target;

  if (fast) {
    m_mode = Mode::SOLVE_FAST;
    m_drive.set_speeds(m_speeds.m_fast_speeds);
    return;
  }

  m_mode = Mode::SOLVE_SLOW;
  m_drive.set_speeds(m_speeds.m_normal_speeds);
}

void Navigator::on_connect_send_feedback() { send_position_direction(); }

void Navigator::send_position_direction() {
  uint8_t data[2] = {uint8_t(m_position), uint8_t(m_direction)};

  Custom_STM_App_Update_Char(CUSTOM_STM_MAZE_MOUSEPOS_CHAR, data);
}
