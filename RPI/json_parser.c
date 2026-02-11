#include "json_parser.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define JSON_MAX_FIELD_LEN 128
#define JSON_MAX_ARRAY_ELEMS 20

// Helper function to find a key and return a pointer to its value start
static const char* find_json_value(const char* json, const char* key) {
    char search_key[JSON_MAX_FIELD_LEN];
    snprintf(search_key, sizeof(search_key), ""%s":", key);
    const char* key_pos = strstr(json, search_key);
    if (!key_pos) return NULL;
    return key_pos + strlen(search_key);
}

// Function to extract an integer value from a JSON string for a given key
int get_json_int(const char *json_string, const char *key, int *value) {
    const char* val_start = find_json_value(json_string, key);
    if (!val_start) return -1;

    // Skip whitespace
    while (*val_start && (*val_start == ' ' || *val_start == '	' || *val_start == '
' || *val_start == '')) {
        val_start++;
    }

    if (sscanf(val_start, "%d", value) == 1) {
        return 0; // Success
    }
    return -1; // Failure
}

// Function to extract a string value from a JSON string for a given key
int get_json_string(const char *json_string, const char *key, char *value_buffer, size_t buffer_size) {
    const char* val_start = find_json_value(json_string, key);
    if (!val_start || *val_start != '"') return -1;

    val_start++; // Move past the opening quote
    const char* val_end = strchr(val_start, '"');
    if (!val_end) return -1;

    size_t len = val_end - val_start;
    if (len >= buffer_size) return -1; // Buffer too small

    strncpy(value_buffer, val_start, len);
    value_buffer[len] = '\0';
    return 0; // Success
}

// Helper for parsing a single obstacle object
static int parse_single_obstacle(const char* obs_str, Obstacle* obstacle) {
    if (get_json_int(obs_str, "id", &obstacle->id) == 0 &&
        get_json_int(obs_str, "x", &obstacle->x) == 0 &&
        get_json_int(obs_str, "y", &obstacle->y) == 0 &&
        get_json_int(obs_str, "d", &obstacle->d) == 0) { // Expect 'd' as integer

        // Adjust coordinates from Android's 1-indexed to RPi's 0-indexed
        obstacle->x -= 1;
        obstacle->y -= 1;

        return 0; // Success
    }
    return -1; // Failure
}

// Function to parse the Android map JSON into the SharedAppContext
int parse_android_map_json(const char* json_string, SharedAppContext* context) {
    const char* obstacles_array_start = strstr(json_string, ""obstacles":[");
    if (!obstacles_array_start) return -1;

    obstacles_array_start += strlen(""obstacles":[");
    const char* obstacles_array_end = strstr(obstacles_array_start, "]");
    if (!obstacles_array_end) return -1;

    char temp_obstacles_str[strlen(json_string) + 1];
    strncpy(temp_obstacles_str, obstacles_array_start, obstacles_array_end - obstacles_array_start);
    temp_obstacles_str[obstacles_array_end - obstacles_array_start] = '\0';

    context->obstacle_count = 0;
    const char* current_obs_str = temp_obstacles_str;
    while (current_obs_str && *current_obs_str != '\0' && context->obstacle_count < MAX_OBSTACLES) {
        const char* obj_start = strchr(current_obs_str, '{');
        if (!obj_start) break;
        const char* obj_end = strchr(obj_start, '}');
        if (!obj_end) break;

        char single_obs_json[JSON_MAX_FIELD_LEN];
        strncpy(single_obs_json, obj_start, obj_end - obj_start + 1);
        single_obs_json[obj_end - obj_start + 1] = '\0';

        if (parse_single_obstacle(single_obs_json, &context->obstacles[context->obstacle_count]) == 0) {
            context->obstacle_count++;
        } else {
            fprintf(stderr, "Error parsing single obstacle JSON: %s\n", single_obs_json);
        }
        current_obs_str = obj_end + 1;
        if (*current_obs_str == ',') current_obs_str++; // Skip comma if present
    }

    // Parse robot_x, robot_y, robot_direction
    int robot_x_val = 1; // Default to 1 (0-indexed)
    int robot_y_val = 1; // Default to 1 (0-indexed)
    int robot_dir_val = 0; // Default to 0 (North)

    if (get_json_int(json_string, "robot_x", &robot_x_val) != 0) {
        fprintf(stderr, "Could not parse robot_x, using default.\n");
    }
    if (get_json_int(json_string, "robot_y", &robot_y_val) != 0) {
        fprintf(stderr, "Could not parse robot_y, using default.\n");
    }

    int android_dir = 0;
    if (get_json_int(json_string, "robot_direction", &android_dir) == 0) {
        // Map Android's 1=N, 2=E, 3=S, 4=W to RPi's 0,2,4,6
        switch (android_dir) {
            case 1: robot_dir_val = 0; break; // North
            case 2: robot_dir_val = 2; break; // East
            case 3: robot_dir_val = 4; break; // South
            case 4: robot_dir_val = 6; break; // West
            default: robot_dir_val = 0; break; // Default to North if unknown or invalid
        }
    } else {
        fprintf(stderr, "Could not parse robot_direction, using default.\n");
    }

    // Adjust coordinates from Android's 1-indexed to RPi's 0-indexed
    context->robot_start_x = robot_x_val - 1;
    context->robot_start_y = robot_y_val - 1;
    context->robot_start_dir = robot_dir_val;

    return 0; // Success
}

// Helper for parsing a single command string (e.g., "FW5000", "SP1")
static int parse_single_command_string(const char* cmd_str, Command* command) {
    if (strncmp(cmd_str, "FW", 2) == 0) {
        command->type = CMD_MOVE_FORWARD;
        command->value = atoi(cmd_str + 2);
    } else if (strncmp(cmd_str, "BW", 2) == 0) {
        command->type = CMD_MOVE_BACKWARD;
        command->value = atoi(cmd_str + 2);
    } else if (strncmp(cmd_str, "FL", 2) == 0) {
        command->type = CMD_TURN_LEFT;
        command->value = atoi(cmd_str + 2); // Assuming angle like 90
    } else if (strncmp(cmd_str, "FR", 2) == 0) {
        command->type = CMD_TURN_RIGHT;
        command->value = atoi(cmd_str + 2); // Assuming angle like 90
    } else if (strncmp(cmd_str, "SP", 2) == 0) {
        command->type = CMD_SNAPSHOT;
        command->value = atoi(cmd_str + 2); // Obstacle ID
    } else {
        fprintf(stderr, "Unknown command type: %s\n", cmd_str);
        return -1; // Unknown command
    }
    return 0;
}

// Function to parse the pathfinding server's route JSON
int parse_route_json(const char* json_string, Command commands[], int* command_count, SnapPosition snap_positions[], int* snap_position_count) {
    // Parse commands array
    const char* commands_array_start = strstr(json_string, "\"commands\":[");
    if (!commands_array_start) return -1;
    commands_array_start += strlen("\"commands\":[");
    const char* commands_array_end = strstr(commands_array_start, "]");
    if (!commands_array_end) return -1;

    // Use a temporary buffer for strtok_r, ensuring it's large enough
    char temp_commands_str_buf[strlen(json_string) + 1];
    strncpy(temp_commands_str_buf, commands_array_start, commands_array_end - commands_array_start);
    temp_commands_str_buf[commands_array_end - commands_array_start] = '\0';
    char* temp_commands_str = temp_commands_str_buf; // Pointer for strtok_r

    *command_count = 0;
    char* saveptr1; // For strtok_r
    char* current_cmd_token = strtok_r(temp_commands_str, ",", &saveptr1);
    while (current_cmd_token != NULL && *command_count < MAX_COMMANDS) {
        // Remove quotes if present
        if (*current_cmd_token == '"') current_cmd_token++;
        char* end_quote = strrchr(current_cmd_token, '"');
        if (end_quote) *end_quote = '\0';

        if (parse_single_command_string(current_cmd_token, &commands[*command_count]) == 0) {
            (*command_count)++;
        } else {
            fprintf(stderr, "Error parsing single command string: %s\n", current_cmd_token);
        }
        current_cmd_token = strtok_r(NULL, ",", &saveptr1);
    }

    // Parse 'path' array and extract snap_positions
    const char* path_array_start = strstr(json_string, "\"path\":[");
    if (!path_array_start) {
        *snap_position_count = 0; // No path found, so no snap positions
        return 0; // Still success if commands were parsed
    }
    path_array_start += strlen("\"path\":[");
    const char* path_array_end = strstr(path_array_start, "]");
    if (!path_array_end) return -1;

    char temp_path_str_buf[strlen(json_string) + 1];
    strncpy(temp_path_str_buf, path_array_start, path_array_end - path_array_start);
    temp_path_str_buf[path_array_end - path_array_start] = '\0';
    char* temp_path_str = temp_path_str_buf; // Pointer for strtok_r

    *snap_position_count = 0;
    char* saveptr2; // For strtok_r
    char* current_path_obj_str = strtok_r(temp_path_str, "}", &saveptr2);
    while (current_path_obj_str != NULL && *snap_position_count < MAX_SNAP_POSITIONS) {
        const char* obj_start = strchr(current_path_obj_str, '{');
        if (obj_start) {
            int x, y, d, s;
            if (get_json_int(obj_start, "x", &x) == 0 &&
                get_json_int(obj_start, "y", &y) == 0 &&
                get_json_int(obj_start, "d", &d) == 0 &&
                get_json_int(obj_start, "s", &s) == 0) {
                
                if (s != -1) { // If it's a snapshot position
                    snap_positions[*snap_position_count] = (SnapPosition){.x = x, .y = y, .d = d};
                    (*snap_position_count)++;
                }
            } else {
                fprintf(stderr, "Error parsing PathPoint object: %s\n", obj_start);
            }
        }
        current_path_obj_str = strtok_r(NULL, "}", &saveptr2);
        if (current_path_obj_str != NULL && *current_path_obj_str == ',') {
            current_path_obj_str++; // Skip comma
        }
    }

    return 0; // Success
}