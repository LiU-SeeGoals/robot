#include "ui.h"

/* Private includes */
#include "log.h"
#include "com.h"
#include "kicker.h"

/* Private defines */
#define RX_BUFFER_LEN 20
#define KEY_NEWLINE   0xd
#define KEY_CTRL_C    0x3
#define KEY_ESC       0x1b
#define KEY_BACKSPACE 0x7f

/* Private enums/structs */
typedef struct {
  int key;
  const char* description;
} CommandInfo;

typedef struct {
  const char* name;
  CommandInfo* cmds;
  size_t len_cmds;
  int parent;
} StructInfo;

// Available states
typedef enum {
  state_default = 0,
  state_kicker,
  state_kicker_edit,
} state;

// Every state has a set of commands.
// [H]elp and [B]ack keys can't be used as they are
// global commands.
CommandInfo default_commands[2] = {
  {'R', "F"},
  {'K', "icker"}
};

CommandInfo kicker_commands[4] = {
  {'C', "harge"},
  {'K', "ick"},
  {'P', "rint vars"},
  {'E', "dit vars"},
};

CommandInfo kicker_edit_commands[3] = {
  {'M', "ax charges per kick"},
  {'C', "harge wait (us)"},
  {'D', "ischarge wait (us)"},
};

// Initializiation of all available states
StructInfo states[3] = {
  {
    .name     = "", 
    .cmds     = default_commands,
    .len_cmds = sizeof(default_commands)/sizeof(CommandInfo),
    .parent   = state_default,
  },
  {
    .name     = "Kicker",
    .cmds     = kicker_commands,
    .len_cmds = sizeof(kicker_commands)/sizeof(CommandInfo),
    .parent   = state_default,
  },
  {
    .name     = "Kicker->Edit",
    .cmds     = kicker_edit_commands,
    .len_cmds = sizeof(kicker_edit_commands)/sizeof(CommandInfo),
    .parent   = state_kicker,
  },
};


/* Private variables */
UART_HandleTypeDef *huart;
state current_state = state_default;
int current_command = -1;
uint8_t key;
uint8_t reading_to_buffer = 0;
uint8_t rx_buffer[RX_BUFFER_LEN];
size_t rx_buffer_loc = 0;
LOG_Module internal_log_mod;

/* Private functions declarations */
void read_to_buffer();
void finished_reading_to_buffer();
void stop_reading_to_buffer();
void parse_key();

/*
 * Public functions implementations
 */

void UI_Init(UART_HandleTypeDef *handle) {
  huart = handle;
  HAL_UART_Receive_IT(huart, &key, 1);
  LOG_InitModule(&internal_log_mod, "UI");
}

void UI_PrintHelp() {
  if (current_state == state_default) {
    LOG_UI("[UI]: [H]elp ");
  } else {
    LOG_UI("[UI->%s]: [B]ack [H]elp\r\n", states[current_state].name);
  }

  for (int i = 0; i < states[current_state].len_cmds; i++) {
    LOG_UI("[%c]%s ", states[current_state].cmds[i].key, states[current_state].cmds[i].description);
  }
  LOG_UI("\r\n");
}

void UI_RxCallback() {
  if (reading_to_buffer) {
    read_to_buffer();
  } else if ((key == 'B' || key == KEY_BACKSPACE) && current_state != state_default) {
    current_state = states[current_state].parent;
    current_command = key;
    UI_PrintHelp();
  } else {
    current_command = key;
    if (key == 'H') {
      UI_PrintHelp();
    } else {
      parse_key();
    }
  }

  HAL_UART_Receive_IT(huart, &key, 1);
}

/*
 * Private functions implementations
 */

void read_to_buffer() {
  if (key == KEY_NEWLINE || rx_buffer_loc == RX_BUFFER_LEN) {
    finished_reading_to_buffer();
    stop_reading_to_buffer();
  } else if (key == KEY_CTRL_C || key == KEY_ESC) {
    LOG_UI("Exiting...\r\n");
    stop_reading_to_buffer();
  } else if (key == KEY_BACKSPACE) {
    if (rx_buffer_loc > 0) {
      rx_buffer_loc--;
      rx_buffer[rx_buffer_loc] = ' ';
    }
    LOG_UI("%s\r\n", rx_buffer);
  } else {
    rx_buffer[rx_buffer_loc] = key;
    rx_buffer_loc++;
    LOG_UI("%s\r\n", rx_buffer);
  }
}

void finished_reading_to_buffer() {
  switch(current_state) {
    case state_kicker_edit:
      switch(current_command) {
        case 'M':
          KICKER_EditValue(KICKER_VAL_MAX_CHARGES_PER_KICK, atoi(rx_buffer));
          break;
        case 'C':
          KICKER_EditValue(KICKER_VAL_CHARGE_WAIT_US, atoi(rx_buffer));
          break;
        case 'D':
          KICKER_EditValue(KICKER_VAL_DISCHARGE_WAIT_US, atoi(rx_buffer));
          break;
      }
      LOG_UI("---\r\n");
      KICKER_PrintValues();
      break;
    default:
      LOG_UI("[UI] Bad state for setting value\r\n");
      break;
  }
}

void stop_reading_to_buffer() {
  reading_to_buffer = 0;
  rx_buffer_loc = 0;
  memset(rx_buffer, 0, RX_BUFFER_LEN);
  UI_PrintHelp();
}

void parse_key() {
  if (current_state == state_default) {
    switch (key) {
      case 'R':
        COM_RF_PrintInfo();
        break;
      case 'K':
        current_state = state_kicker;
        UI_PrintHelp();
        break;
      default:
        LOG_UI("Unknown command: %c (%x)\r\n", key, key);
        UI_PrintHelp();
        break;
    }
  } else if (current_state == state_kicker) {
    switch (key) {
      case 'C':
        KICKER_Charge();
        break;
      case 'K':
        KICKER_Kick();
        break;
      case 'P':
        KICKER_PrintValues();
        break;
      case 'E':
        current_state = state_kicker_edit;
        UI_PrintHelp();
        break;
      default:
        UI_PrintHelp();
        break;
    }
  } else if (current_state == state_kicker_edit) {
    switch (key) {
      case 'M':
        reading_to_buffer = 1;
        LOG_UI("New value: \r\n");
        break;
      case 'C':
        reading_to_buffer = 1;
        LOG_UI("New value: \r\n");
        break;
      case 'D':
        reading_to_buffer = 1;
        LOG_UI("New value: \r\n");
        break;
      default:
        UI_PrintHelp();
        break;
    }
  }
}
