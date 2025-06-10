#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "st_shell.h"
#include "st_shell_config.h"

static void sti_shell_help_command();
static void sti_shell_print_arg_help(st_shell_command_entry_t *command);

char st_shell_input_command_buffer[256] = {'\0'};
static st_shell_command_entry_t **command_database = {0};

void st_shell_init()
{
    for (int i=0; i<ST_SHELL_COMMAND_MAX_LENGTH; i++) {
        st_shell_input_command_buffer[i] = '\0';
    }
    while (true) {
        int char_pos = 0;
        char input_char = '\0';
        while (input_char != '\n' && char_pos < ST_SHELL_COMMAND_MAX_LENGTH) {
            input_char = getchar();
            st_shell_input_command_buffer[char_pos] = input_char;
            char_pos++;
        }
        if (char_pos == ST_SHELL_COMMAND_MAX_LENGTH) {            
            continue;
        }
        st_shell_command_t input_command = {0};
        st_shell_input_command_buffer[char_pos - 1] = '\0';
        st_shell_parse_command(st_shell_input_command_buffer, &input_command);
        st_shell_execute_command(&input_command);
        free(input_command.name);
        for (int i=0; i<input_command.argc; i++) {
            free(input_command.argv[i]);
        }
        free(input_command.argv);
    }
}

void st_shell_parse_command(char *buffer, st_shell_command_t *input_command)
{
    uint16_t length = (uint16_t)strlen(buffer);
    uint8_t arg_count = 0;
    for (uint8_t i=0; i<length; i++) {
        if (buffer[i] == ' ') {
            arg_count++;
        }
    }
    char *token = strtok(buffer, ST_SHELL_ARG_SEPERATOR);
    char *command_name = (char*)calloc(strlen(token)+1, sizeof(char));
    char **argv = (char**)calloc(arg_count, sizeof(char*));
    strcpy(command_name, token);
    token = strtok(NULL, ST_SHELL_ARG_SEPERATOR);
    int arg_pos = 0;
    while (token != NULL) {
        char *arg = (char*)calloc(strlen(token) + 1, sizeof(char));
        strcpy(arg, token);
        argv[arg_pos] = arg;
        arg_pos++;
        token = strtok(NULL, ST_SHELL_ARG_SEPERATOR);
    }
    input_command->name = command_name;
    input_command->argc = arg_count;
    input_command->argv = argv;
}

void st_shell_add_command_group(st_shell_command_entry_t** command_group)
{
    command_database = command_group;
    int i=0;
    while (command_database[i] != NULL) {
        i++;
    }
}

void st_shell_execute_command(st_shell_command_t *command)
{
    int i=0;
    bool is_match_found = false;
    if (!strcmp(command->name, "help")) {
        sti_shell_help_command();
        return;
    }
    while (command_database[i] != NULL) {
        if(!strcmp(command->name, command_database[i]->command)) {
            if (command->argc == command_database[i]->arg_count) {
                command_database[i]->handler(command);
            } else {
                printf("%s\n", command->name);
                sti_shell_print_arg_help(command_database[i]);
            }
            is_match_found = true;
            break;
        } else if (!strncmp(command->name, command_database[i]->command, strlen(command->name))){
            printf("%s\n", command_database[i]->command);
            sti_shell_print_arg_help(command_database[i]);
            is_match_found = true;
        }
        i++;
    }
    if (!is_match_found) {
        printf("Command Not Found\n");
    }
}

static void sti_shell_help_command()
{
    int i=0;
    while (command_database[i] != NULL) {
        printf("%s: %s\n", command_database[i]->command, command_database[i]->help_text);
        sti_shell_print_arg_help(command_database[i]);
        i++;
    }
}

static void sti_shell_print_arg_help(st_shell_command_entry_t *command)
{
    if (command->arg_help == NULL) {
        printf("No argument help available.\n");
        return;
    }

    char arg_help_copy[100];
    strncpy(arg_help_copy, command->arg_help, sizeof(arg_help_copy));
    arg_help_copy[sizeof(arg_help_copy) - 1] = '\0';
    char *token = strtok(arg_help_copy, ST_SHELL_ARG_HELP_SEPERATOR);
    printf("%5s", "");
    while (token != NULL) {
        printf("<%s> ", token);
        token = strtok(NULL, ST_SHELL_ARG_HELP_SEPERATOR);
    }
    printf("\n");
}