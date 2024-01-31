#include "debug_ui.h"

/* Private includes */
#include "com.h"
#include "kicker.h"

/* Private defines */
#define RX_BUFFER_LEN   20
#define KEY_NEWLINE     0xd
#define KEY_CTRL_C      0x3
#define KEY_ESC         0x1b

/* Private enums/structs */
typedef struct {
  int cmd;
  const char* description;
} CommandInfo;

typedef struct {
  int id;
  int parent;
  const char* name;
  CommandInfo* cmds;
  size_t len_cmds;
} StructInfo;

// Available states and commands.
typedef enum {
  state_default = 0,
  state_kicker,
  state_kicker_edit,
} state;

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
    .id       = state_default,
    .parent   = state_default,
    .name     = "", 
    .cmds     = default_commands,
    .len_cmds = sizeof(default_commands)/sizeof(CommandInfo),
  },
  {
    .id       = state_kicker,
    .parent   = state_default,
    .name     = "Kicker",
    .cmds     = kicker_commands,
    .len_cmds = sizeof(kicker_commands)/sizeof(CommandInfo),
  },
  {
    .id       = state_kicker_edit,
    .parent   = state_kicker,
    .name     = "Kicker-Edit",
    .cmds     = kicker_edit_commands,
    .len_cmds = sizeof(kicker_edit_commands)/sizeof(CommandInfo),
  },
};


/* Private variables */
UART_HandleTypeDef *huart;
state current_state = state_default;
int current_command = -1;
uint8_t key;
uint8_t reading_user_value = 0;
uint8_t rx_buffer[RX_BUFFER_LEN];
size_t rx_buffer_loc = 0;

/* Private functions declarations */
void read_to_buffer();
void set_value_from_buffer();
void stop_reading_user();
void parse_key();

/*
 * Public functions implementations
 */

void DEBUG_UI_Init(UART_HandleTypeDef *handle) {
  huart = handle;
  HAL_UART_Receive_IT(huart, &key, 1);
}

void DEBUG_UI_PrintHelp() {
  if (current_state == state_default) {
    printf("[UI]: ");
    printf("[H]elp ");
  } else {
    printf("[UI-%s]: ", states[current_state].name);
    printf("[B]ack ");
  }

  for (int i = 0; i < states[current_state].len_cmds; i++) {
    printf("[%c]%s ", states[current_state].cmds[i].cmd, states[current_state].cmds[i].description);
  }
  printf("\r\n");
}

void DEBUG_UI_RxCallback() {
  if (reading_user_value) {
    read_to_buffer();
  } else if (key == 'B' && current_state != state_default) {
    current_state = states[current_state].parent;
    current_command = key;
    DEBUG_UI_PrintHelp();
  } else {
    current_command = key;
    if (key == 'H') {
      DEBUG_UI_PrintHelp();
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
    set_value_from_buffer();
    stop_reading_user();
  } else if (key == KEY_CTRL_C || key == KEY_ESC) {
    printf("Not setting anything...\r\n");
    stop_reading_user();
  } else {
    rx_buffer[rx_buffer_loc] = key;
    rx_buffer_loc++;
    printf("New value: %s\r\n", rx_buffer);
  }
}

void set_value_from_buffer() {
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
      printf("---\r\n");
      KICKER_PrintValues();
      break;
    default:
      printf("[UI] Bad state for setting value\r\n");
      break;
  }
}

void stop_reading_user() {
  reading_user_value = 0;
  rx_buffer_loc = 0;
  memset(rx_buffer, 0, RX_BUFFER_LEN);
  current_state = states[current_state].parent;
  DEBUG_UI_PrintHelp();
}

void parse_key() {
  if (current_state == state_default) {
    switch (key) {
      case 'R':
        COM_RF_PrintInfo();
        break;
      case 'K':
        current_state = state_kicker;
        DEBUG_UI_PrintHelp();
        break;
      default:
        printf("Unknown command: %c (%x)\r\n", key, key);
        DEBUG_UI_PrintHelp();
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
        DEBUG_UI_PrintHelp();
        break;
      default:
        DEBUG_UI_PrintHelp();
        break;
    }
  } else if (current_state == state_kicker_edit) {
    switch (key) {
      case 'M':
        reading_user_value = 1;
        printf("New value: \r\n");
        break;
      case 'C':
        reading_user_value = 1;
        printf("New value: \r\n");
        break;
      case 'D':
        reading_user_value = 1;
        printf("New value: \r\n");
        break;
      default:
        DEBUG_UI_PrintHelp();
        break;
    }
  }
}
