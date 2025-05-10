#include <graphx.h>
#include <keypadc.h>
#include <srldrvce.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/timers.h>
#include <tice.h>

enum {
  COLOR_BLACK = 0,
  COLOR_WHITE = 1,
  COLOR_LIGHT_LIGHT_GRAY = 2,
  COLOR_LIGHT_GRAY = 3,
  COLOR_GRAY = 4,

  COLOR_GREEN = 0x06,
  COLOR_RED = 0xE1,
  COLOR_BLUE = 0x1B,
  COLOR_NONE = 0xFF,
};

////////////////////////////////////////////////////////////////////////////////
/// USB STUFF
////////////////////////////////////////////////////////////////////////////////

srl_device_t srl;
bool has_srl_device = false;
uint8_t srl_buf[512];

// From srl_echo example
static usb_error_t handle_usb_event(usb_event_t event, void *event_data,
                                    usb_callback_data_t *callback_data) {

  usb_error_t err;

  err = srl_UsbEventCallback(event, event_data, callback_data);
  if (err != USB_SUCCESS)
    return err;

  if (event == USB_DEVICE_CONNECTED_EVENT &&
      !(usb_GetRole() & USB_ROLE_DEVICE)) {
    usb_device_t device = event_data;
    usb_ResetDevice(device);
  }

  if (event == USB_HOST_CONFIGURE_EVENT ||
      (event == USB_DEVICE_ENABLED_EVENT &&
       !(usb_GetRole() & USB_ROLE_DEVICE))) {

    if (has_srl_device)
      return USB_SUCCESS;

    usb_device_t device;
    if (event == USB_HOST_CONFIGURE_EVENT) {
      device = usb_FindDevice(NULL, NULL, USB_SKIP_HUBS);
      if (device == NULL)
        return USB_SUCCESS;
    } else {
      device = event_data;
    }

    srl_error_t error = srl_Open(&srl, device, srl_buf, sizeof srl_buf,
                                 SRL_INTERFACE_ANY, 9600);
    if (error)
      return USB_SUCCESS;

    has_srl_device = true;
  }

  if (event == USB_DEVICE_DISCONNECTED_EVENT) {
    usb_device_t device = event_data;
    if (device == srl.dev) {
      srl_Close(&srl);
      has_srl_device = false;
    }
  }

  return USB_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
/// USER CONTROL INPUT
////////////////////////////////////////////////////////////////////////////////

typedef union {
  uint8_t data;
  struct {
    uint8_t forward : 1;
    uint8_t backward : 1;
    uint8_t turn_cw : 1;
    uint8_t turn_ccw : 1;
    uint8_t speed : 4;
  };
} control_data_t;

#define DIRECTIONS_MASK 0x0F
#define SPEED_MASK 0xF0

bool get_speed_input(control_data_t *data) {
  uint8_t last_speed = data->speed;

  if (kb_IsDown(kb_KeyGraph)) {
    data->speed = 5;
  } else if (kb_IsDown(kb_KeyTrace)) {
    data->speed = 4;
  } else if (kb_IsDown(kb_KeyZoom)) {
    data->speed = 3;
  } else if (kb_IsDown(kb_KeyWindow)) {
    data->speed = 2;
  } else if (kb_IsDown(kb_KeyYequ)) {
    data->speed = 1;
  }

  return (data->speed != last_speed);
}

bool get_direction_input(control_data_t *data) {
  uint8_t last_directions = data->data & DIRECTIONS_MASK;
  data->data &= ~DIRECTIONS_MASK;

  bool up = kb_IsDown(kb_KeyUp);
  bool down = kb_IsDown(kb_KeyDown);
  bool left = kb_IsDown(kb_KeyLeft);
  bool right = kb_IsDown(kb_KeyRight);

  if (up != down) {
    if (up)
      data->forward = 1;
    else
      data->backward = 1;
  }

  if (left != right) {
    if (left)
      data->turn_ccw = 1;
    else
      data->turn_cw = 1;
  }

  uint8_t new_directions = data->data & DIRECTIONS_MASK;
  return (new_directions != last_directions);
}

////////////////////////////////////////////////////////////////////////////////
/// GRAPHICS
////////////////////////////////////////////////////////////////////////////////

#define TEXT_1X_HEIGHT 8
#define TEXT_2X_HEIGHT 16

#define TEXT_PADDING 4

#define TITLEBAR_HEIGHT (TEXT_1X_HEIGHT + 2 * TEXT_PADDING)

#define SPEED_BUTTON_HEIGHT (TEXT_1X_HEIGHT + 2 * TEXT_PADDING)
#define SPEED_BUTTON_WIDTH ((GFX_LCD_WIDTH - TEXT_PADDING) / 5 - TEXT_PADDING)

#define SPEED_BAR_HEIGHT                                                       \
  (SPEED_BUTTON_HEIGHT + TEXT_PADDING * 2 + TEXT_1X_HEIGHT)

#define CENTER_HEIGHT (GFX_LCD_HEIGHT - TITLEBAR_HEIGHT - SPEED_BAR_HEIGHT)

#define ARROW_BODY_WIDTH 4
#define ARROW_HEIGHT 23
#define ARROW_HEAD_OVERHANG 3
#define ARROW_HEAD_HEIGHT 7
#define ARROW_BODY_HEIGHT (ARROW_HEIGHT - ARROW_HEAD_HEIGHT)

void center_text(const char *str, int x_offset, int region_width, int y) {
  gfx_PrintStringXY(str,
                    x_offset + (region_width - gfx_GetStringWidth(str)) / 2, y);
}

void draw_titlebar(void) {
  gfx_SetTextScale(1, 1);
  gfx_SetTextFGColor(COLOR_WHITE);
  gfx_SetTextBGColor(COLOR_NONE);

  const char *title_str = "MicroMouse";

  const char *status_str;
  if (has_srl_device) {
    gfx_SetColor(COLOR_GREEN);
    status_str = "Connected";
  } else {
    gfx_SetColor(COLOR_RED);
    status_str = "Disconnected";
  }

  int status_width = gfx_GetStringWidth(status_str);
  gfx_FillRectangle_NoClip(0, 0, status_width + 2 * TEXT_PADDING,
                           TITLEBAR_HEIGHT);
  gfx_PrintStringXY(status_str, TEXT_PADDING, TEXT_PADDING);

  gfx_SetColor(COLOR_BLUE);
  gfx_FillRectangle_NoClip(status_width + 2 * TEXT_PADDING, 0,
                           GFX_LCD_WIDTH - status_width - 2 * TEXT_PADDING,
                           TITLEBAR_HEIGHT);

  center_text(title_str, 0, GFX_LCD_WIDTH, TEXT_PADDING);
}

void draw_linear(control_data_t data) {
  gfx_SetTextFGColor(COLOR_BLACK);
  gfx_SetTextBGColor(COLOR_NONE);

  // Title Text

  const char *title_str = "Linear";

  gfx_SetTextScale(2, 2);
  center_text(title_str, 0, GFX_LCD_WIDTH / 2,
              CENTER_HEIGHT / 4 + TITLEBAR_HEIGHT - TEXT_2X_HEIGHT / 2);

  // Visual

  gfx_SetColor(COLOR_BLACK);

  int center_x = GFX_LCD_WIDTH / 4;
  int center_y = TITLEBAR_HEIGHT + CENTER_HEIGHT / 2;

  int arrow_body_left_x = center_x - ARROW_BODY_WIDTH / 2;
  int arrow_body_right_x = center_x + ARROW_BODY_WIDTH / 2;

  const char *status_str;
  if (data.forward) {
    status_str = "Forwards";

    // Up arrow

    int arrow_top_y = center_y - ARROW_HEIGHT / 2;
    int arrow_body_top_y = arrow_top_y + ARROW_HEAD_HEIGHT;

    gfx_FillRectangle_NoClip(arrow_body_left_x, arrow_body_top_y,
                             ARROW_BODY_WIDTH, ARROW_BODY_HEIGHT);

    gfx_FillTriangle_NoClip(
        arrow_body_left_x - ARROW_HEAD_OVERHANG, arrow_body_top_y, // 1
        center_x, arrow_top_y,                                     // 2
        arrow_body_right_x + ARROW_HEAD_OVERHANG, arrow_body_top_y // 3
    );

  } else if (data.backward) {
    status_str = "Backwards";

    // Down arrow

    int arrow_body_top_y = center_y - ARROW_HEIGHT / 2;
    int arrow_bottom_y = center_y + ARROW_HEIGHT / 2;
    int arrow_body_bottom_y = arrow_bottom_y - ARROW_HEAD_HEIGHT;

    gfx_FillRectangle_NoClip(arrow_body_left_x, arrow_body_top_y,
                             ARROW_BODY_WIDTH, ARROW_BODY_HEIGHT);

    gfx_FillTriangle_NoClip(
        arrow_body_left_x - ARROW_HEAD_OVERHANG, arrow_body_bottom_y, // 1
        center_x, arrow_bottom_y,                                     // 2
        arrow_body_right_x + ARROW_HEAD_OVERHANG, arrow_body_bottom_y // 3
    );

  } else {
    status_str = "None";

    center_text("X", 0, GFX_LCD_WIDTH / 2, center_y - TEXT_2X_HEIGHT / 2);
  }

  // Status Text

  int lower_y = 3 * (CENTER_HEIGHT / 4) + TITLEBAR_HEIGHT - TEXT_1X_HEIGHT / 2;

  gfx_SetTextScale(1, 1);
  center_text(status_str, 0, GFX_LCD_WIDTH / 2, lower_y);
}

void draw_angular(control_data_t data) {
  gfx_SetTextFGColor(COLOR_BLACK);
  gfx_SetTextBGColor(COLOR_NONE);

  // Title Text

  const char *title_str = "Angular";

  gfx_SetTextScale(2, 2);
  center_text(title_str, GFX_LCD_WIDTH / 2, GFX_LCD_WIDTH / 2,
              CENTER_HEIGHT / 4 + TITLEBAR_HEIGHT - TEXT_2X_HEIGHT / 2);

  // TODO: Visual

  // Status Text

  const char *status_str = "None";
  if (data.turn_cw) {
    status_str = "CW";
  } else if (data.turn_ccw) {
    status_str = "CCW";
  }

  int lower_y = 3 * (CENTER_HEIGHT / 4) + TITLEBAR_HEIGHT - TEXT_1X_HEIGHT / 2;

  gfx_SetTextScale(1, 1);
  center_text(status_str, GFX_LCD_WIDTH / 2, GFX_LCD_WIDTH / 2, lower_y);
}

void draw_directions(control_data_t data) {
  draw_linear(data);
  draw_angular(data);

  gfx_SetColor(COLOR_LIGHT_LIGHT_GRAY);
  gfx_Line(GFX_LCD_WIDTH / 2, TITLEBAR_HEIGHT, GFX_LCD_WIDTH / 2,
           GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT);
}

void draw_speed_buttons(control_data_t data) {
  int speed = data.speed;

  gfx_SetTextScale(1, 1);

  {
    gfx_SetColor(COLOR_LIGHT_LIGHT_GRAY);

    gfx_Line(0, GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT, GFX_LCD_WIDTH,
             GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT);
    int center_line_y =
        GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT + TEXT_PADDING + TEXT_1X_HEIGHT / 2;
    gfx_Line(0, center_line_y, GFX_LCD_WIDTH, center_line_y);

    gfx_SetTextBGColor(COLOR_WHITE);
    gfx_SetTextFGColor(COLOR_GRAY);

    center_text("    Speed    ", 0, GFX_LCD_WIDTH,
                GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT + TEXT_PADDING);
  }

  gfx_SetTextBGColor(COLOR_NONE);
  gfx_SetTextFGColor(COLOR_WHITE);

  char number[2] = {0, 0};

  for (int i = 0; i < 5; ++i) {
    int x = TEXT_PADDING * (i + 1) + SPEED_BUTTON_WIDTH * i;
    int y = GFX_LCD_HEIGHT - SPEED_BUTTON_HEIGHT;

    gfx_SetColor((i + 1 == speed) ? COLOR_GRAY : COLOR_LIGHT_GRAY);
    gfx_FillRectangle_NoClip(x, y, SPEED_BUTTON_WIDTH, SPEED_BUTTON_HEIGHT);

    gfx_SetColor(COLOR_GRAY);
    gfx_Rectangle_NoClip(x, y, SPEED_BUTTON_WIDTH, SPEED_BUTTON_HEIGHT);

    number[0] = '1' + i;
    center_text(number, x, SPEED_BUTTON_WIDTH, y + TEXT_PADDING);
  }
}

void initial_draw(void) {
  draw_titlebar();

  control_data_t data = {0};
  data.speed = 3;

  draw_directions(data);
  draw_speed_buttons(data);

  gfx_SwapDraw();
}

void update_graphics(bool directions_changed, bool speed_changed,
                     control_data_t data, bool connection_changed) {
  if (directions_changed || speed_changed || connection_changed) {
    gfx_FillScreen(COLOR_WHITE);
  }

  if (directions_changed) {
    draw_directions(data);
    gfx_BlitRectangle(gfx_buffer, 0, TITLEBAR_HEIGHT, GFX_LCD_WIDTH,
                      CENTER_HEIGHT);
  }

  if (speed_changed) {
    draw_speed_buttons(data);
    gfx_BlitRectangle(gfx_buffer, 0, GFX_LCD_HEIGHT - SPEED_BAR_HEIGHT,
                      GFX_LCD_WIDTH, SPEED_BAR_HEIGHT);
  }

  if (connection_changed) {
    draw_titlebar();
    gfx_BlitRectangle(gfx_buffer, 0, 0, GFX_LCD_WIDTH, TITLEBAR_HEIGHT);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION
////////////////////////////////////////////////////////////////////////////////

int main(void) {
  os_ClrHome();

  const usb_standard_descriptors_t *desc = srl_GetCDCStandardDescriptors();
  usb_error_t usb_error =
      usb_Init(handle_usb_event, NULL, desc, USB_DEFAULT_INIT_FLAGS);
  if (usb_error) {
    usb_Cleanup();
    printf("usb init error %u\n", usb_error);
    do
      kb_Scan();
    while (!kb_IsDown(kb_KeyClear));
    return 1;
  }

  gfx_Begin();

  gfx_palette[COLOR_WHITE] = gfx_RGBTo1555(255, 255, 255);
  gfx_palette[COLOR_LIGHT_LIGHT_GRAY] = gfx_RGBTo1555(224, 224, 224);
  gfx_palette[COLOR_LIGHT_GRAY] = gfx_RGBTo1555(192, 192, 192);
  gfx_palette[COLOR_GRAY] = gfx_RGBTo1555(128, 128, 128);

  gfx_SetDrawBuffer();

  control_data_t data = {.speed = 3};
  bool quit = false;

  initial_draw();
  update_graphics(false, true, data, false);

  while (!quit) {
    kb_Scan();

    bool had_srl_device = has_srl_device;
    usb_HandleEvents();

    bool speed_changed = get_speed_input(&data);
    bool directions_changed = get_direction_input(&data);
    bool data_changed = speed_changed || directions_changed;

    if ((quit = kb_IsDown(kb_KeyMode))) {
      // On quit, send one last byte with zero data before exiting.
      data.data &= ~DIRECTIONS_MASK;
    }

    // Skip if already sent zero.
    bool skip_write = (!(data.data & DIRECTIONS_MASK) && !data_changed);

    update_graphics(directions_changed, speed_changed, data,
                    had_srl_device != has_srl_device);

    if (has_srl_device && !skip_write) {
      // Send control data
      srl_Write(&srl, &data, 1);
    }

    usleep(20000); // 20ms
  }

  gfx_End();

  usb_Cleanup();
  return 0;
}
