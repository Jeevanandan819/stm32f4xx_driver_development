#include <stdint.h>

typedef enum
{
    ST_SHELL_ARG_TYPE_UINT8,
    ST_SHELL_ARG_TYPE_UINT16,
    ST_SHELL_ARG_TYPE_UINT32,
    ST_SHELL_ARG_TYPE_INT8,
    ST_SHELL_ARG_TYPE_INT16,
    ST_SHELL_ARG_TYPE_INT32,
    ST_SHELL_ARG_TYPE_HEX,
    ST_SHELL_ARG_TYPE_CHAR,
    ST_SHELL_ARG_TYPE_STRING,
    ST_SHELL_ARG_TYPE_LAST
} st_shell_arg_type_t;

typedef struct
{
    char *name;
    char **argv;
    int argc;
} st_shell_command_t;

typedef void (*st_shell_command_handler_t)(st_shell_command_t *command);

typedef struct {
    char *command;
    char *help_text;
    char *arg_help;
    int arg_count;
    st_shell_arg_type_t *arg_types;
    st_shell_command_handler_t handler;
} st_shell_command_entry_t;

void st_shell_init();

void st_shell_parse_command(char *buffer, st_shell_command_t *input_command);

void st_shell_execute_command(st_shell_command_t *command);

void st_shell_add_command_group(st_shell_command_entry_t** command_group);