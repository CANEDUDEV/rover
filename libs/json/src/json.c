#include "json.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arena.h"

struct json_arena {
  arena_t arena;
  char buf[JSON_MAX_SIZE];
};

static struct json_arena json_arena;

#define PARSER_BUFSIZE 64

// Parser state tracker
typedef struct {
  char buf[PARSER_BUFSIZE];
  const char *ptr;
  json_object_t *current;
  bool in_array;
} json_parser_t;

// Helpers
int parse_object(json_parser_t *parser);
int parse_value(json_parser_t *parser);
int parse_object_name(json_parser_t *parser);
char *parse_string(json_parser_t *parser);
int json_strlen(const char *str);
json_object_t *get_parent(json_object_t *object);
bool is_empty(json_object_t *object);
void new_child_object(json_parser_t *parser);
void new_object(json_parser_t *parser);

json_t *json_parse(const char *data) {
  json_arena.arena = arena_init(json_arena.buf, JSON_MAX_SIZE);

  json_t *json = arena_alloc(&json_arena.arena, sizeof(json_t));
  if (!json) {
    printf("error: out of memory.\r\n");
    return NULL;
  }

  json->root.prev = NULL;

  json_parser_t parser = {
      .ptr = data,
      .current = &json->root,
      .in_array = false,
  };

  if (parse_object(&parser) < 0) {
    printf("error: failed to parse object\r\n");
    return NULL;
  }

  return json;
}

int parse_object(json_parser_t *parser) {
  json_object_t *root = parser->current;

  while (parser->current != NULL) {
    int bytes_read = 0;
    sscanf(parser->ptr, "%1s %n", parser->buf, &bytes_read);
    if (bytes_read < 1) {
      // Reached EOF
      return 0;
    }
    parser->ptr += bytes_read;

    // Handle object
    switch (parser->buf[0]) {
      case '{':  // Start of object
        parser->current->type = JSON_OBJECT;
        parser->in_array = false;
        new_child_object(parser);
        continue;

      case '[':  // Start of array
        parser->current->type = JSON_ARRAY;
        parser->in_array = true;
        new_child_object(parser);
        continue;

      case '\"':  // Start of object name, or string value in array
        if (parser->in_array) {
          parser->ptr--;
        } else if (parse_object_name(parser) < 0) {
          return -1;
        }
        break;

      case ',':  // Next object or value
        new_object(parser);
        continue;

      case '}':  // End of object or array
      case ']': {
        json_object_t *parent = get_parent(parser->current);

        // Reached end of object
        if (parent == root) {
          return 0;
        }

        // Fill in correct type if object is empty
        if (is_empty(parser->current)) {
          parser->current->type = parent->type;
        }

        // In arrays, objects have no name
        parser->in_array = false;
        if (!parent->name) {
          parser->in_array = true;
        }

        parser->current = parent;

        continue;
      }

      default:  // Either we are in an array, or something is wrong.
        if (!parser->in_array) {
          printf("error: invalid JSON\r\n");
          return -1;
        }

        parser->ptr -= bytes_read;
        break;
    }

    if (parse_value(parser) < 0) {
      return -1;
    }
  }

  // We end up here if new_object() or new_child_object() return NULL,
  // i.e. we run out of memory
  return -1;
}

json_object_t *json_get_object(const char *name, json_object_t *root) {
  json_object_t *current = root->child;
  json_object_t *found = NULL;
  while (current != NULL) {
    // Last value wins
    if (current->name && strcmp(current->name, name) == 0) {
      found = current;
    }
    current = current->next;
  }
  return found;
}

int json_insert_object(const char *json, json_object_t *root) {
  if (!root->child || root->child->type != JSON_OBJECT) {
    printf("error: cannot insert object: root is not a JSON object\r\n");
    return -1;
  }

  json_parser_t parser = {
      .ptr = json,
      .current = root->child,
      .in_array = false,
  };

  while (parser.current->next != NULL) {
    parser.current = parser.current->next;
  }

  new_object(&parser);  // Out of memory handled in parse_object

  if (parse_object(&parser) < 0) {
    printf("error: failed to parse object\r\n");
    return -1;
  }

  return 0;
}

int parse_value(json_parser_t *parser) {
  int bytes_read = 0;

  // Skip whitespace
  sscanf(parser->ptr, "%*[\n\r\t ] %n", &bytes_read);
  parser->ptr += bytes_read;

  // Determine value type
  if (sscanf(parser->ptr, "%c", parser->buf) < 1) {
    printf("error: invalid value: %c\r\n", *parser->buf);
    return -1;
  }

  json_value_t value;

  // Parse value
  switch (parser->buf[0]) {
      // Handle in json_parse()
    case '{':
    case '[':
      return 0;

    case '\"':  // String
      parser->current->type = JSON_STRING;
      parser->ptr++;
      value.string = parse_string(parser);
      if (!value.string) {
        return -1;
      }
      break;

    case 't':  // true
      if (sscanf(parser->ptr, "%[^] \n\r\t , } ] %n", parser->buf,
                 &bytes_read) < 1 ||
          strcmp(parser->buf, "true") != 0) {
        printf("error: invalid value: %s\r\n", parser->buf);
        return -1;
      }
      parser->current->type = JSON_BOOL;
      value.boolean = true;
      parser->ptr += bytes_read;
      break;

    case 'f':  // false
      if (sscanf(parser->ptr, "%[^] \n\r\t , } ] %n", parser->buf,
                 &bytes_read) < 1 ||
          strcmp(parser->buf, "false") != 0) {
        printf("error: invalid value: %s\r\n", parser->buf);
        return -1;
      }
      parser->current->type = JSON_BOOL;
      value.boolean = false;
      parser->ptr += bytes_read;
      break;

    case 'n':  // null
      if (sscanf(parser->ptr, "%[^] \n\r\t , } ] %n", parser->buf,
                 &bytes_read) < 1 ||
          strcmp(parser->buf, "null") != 0) {
        printf("error: invalid value: %s\r\n", parser->buf);
        return -1;
      }
      parser->current->type = JSON_NULL;
      parser->current->value = NULL;
      parser->ptr += bytes_read;
      return 0;

    default:  // Number
    {
      if (sscanf(parser->ptr, "%[^] \n\r\t , } ] %n", parser->buf,
                 &bytes_read) < 1) {
        printf("error: invalid JSON\r\n");
        return -1;
      }

      char *err = "";

      if (strchr(parser->buf, '.')) {
        parser->current->type = JSON_FLOAT;
        value.float_ = strtof(parser->buf, &err);
      } else {
        parser->current->type = JSON_INT;
        // NOLINTNEXTLINE(*-magic-numbers)  // Base 10 number
        value.int_ = (int)strtol(parser->buf, &err, 10);
      }
      if (*err) {
        printf("error: invalid JSON\r\n");
        return -1;
      }

      parser->ptr += bytes_read;

      break;
    }
  }

  // Allocate value struct
  parser->current->value = arena_alloc(&json_arena.arena, sizeof(json_value_t));
  if (!parser->current->value) {
    printf("error: out of memory.\r\n");
    return -1;
  }

  *parser->current->value = value;

  return 0;
}

int parse_object_name(json_parser_t *parser) {
  char *str = parse_string(parser);
  if (!str) {
    return -1;
  }

  parser->current->name = str;

  int bytes_read = 0;

  // Check colon and skip whitespace
  if (sscanf(parser->ptr, "%[\n\r\t : ] %n", parser->buf, &bytes_read) < 1 ||
      !strchr(parser->buf, ':')) {
    printf("error: no colon after name.\r\n");
    return -1;
  }

  parser->ptr += bytes_read;
  return 0;
}

char *parse_string(json_parser_t *parser) {
  size_t str_len = json_strlen(parser->ptr);
  char *str = arena_alloc(&json_arena.arena, str_len + 1);
  if (!str) {
    printf("error: out of memory.\r\n");
    return NULL;
  }
  strncpy(str, parser->ptr, str_len);
  str[str_len] = '\0';
  parser->ptr += str_len + 1;  // Go to char just after the string
  return str;
}

int json_strlen(const char *str) {
  const char *ptr = str;

  if (*ptr == '\"') {
    return 0;
  }

  while (++ptr) {
    if (*ptr == '\"' && *(ptr - 1) != '\\') {
      break;
    }
  }

  return (int)(ptr - str);
}

bool is_empty(json_object_t *object) {
  return !object->child && !object->value && object->type != JSON_NULL;
}

json_object_t *get_parent(json_object_t *object) {
  json_object_t *current = object;
  while (true) {
    if (!current->prev) {
      return NULL;
    }
    if (current->prev->child == current) {
      return current->prev;
    }
    current = current->prev;
  }
}

void new_child_object(json_parser_t *parser) {
  parser->current->child =
      arena_alloc(&json_arena.arena, sizeof(json_object_t));
  if (!parser->current->child) {
    printf("error: out of memory.\r\n");
    parser->current = NULL;
    return;
  }
  parser->current->child->prev = parser->current;
  parser->current = parser->current->child;
}

void new_object(json_parser_t *parser) {
  parser->current->next = arena_alloc(&json_arena.arena, sizeof(json_object_t));
  if (!parser->current->next) {
    printf("error: out of memory.\r\n");
    parser->current = NULL;
    return;
  }
  parser->current->next->prev = parser->current;
  parser->current = parser->current->next;
}

// print functions for testing and debugging

int json_sprint_value(json_object_t *current, char *str) {
  switch (current->type) {
    case JSON_BOOL:
      if (current->value->boolean == true) {
        return sprintf(str, "true");
      } else {
        return sprintf(str, "false");
      }

    case JSON_STRING:
      return sprintf(str, "\"%s\"", current->value->string);

    case JSON_INT:
      return sprintf(str, "%d", current->value->int_);

    case JSON_FLOAT:
      return sprintf(str, "%f", current->value->float_);

    case JSON_NULL:
      return sprintf(str, "null");

    default:
      return 0;
  }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void json_sprint(json_t *json, char *str) {
  json_object_t *current = &json->root;

  while (current != NULL) {
    if (current->name) {
      str += sprintf(str, "\"%s\":", current->name);
    }

    switch (current->type) {
      case JSON_OBJECT:
        str += sprintf(str, "{");
        if (is_empty(current->child)) {
          str += sprintf(str, "}");
          break;
        }
        current = current->child;
        continue;

      case JSON_ARRAY:
        str += sprintf(str, "[");
        if (is_empty(current->child)) {
          str += sprintf(str, "]");
          break;
        }
        current = current->child;
        continue;

      default:
        str += json_sprint_value(current, str);
        break;
    }

    // Close objects and arrays if needed
    while (!current->next) {
      json_object_t *parent = get_parent(current);
      if (!parent) {
        break;
      }
      if (parent->type == JSON_OBJECT) {
        str += sprintf(str, "}");
      }
      if (parent->type == JSON_ARRAY) {
        str += sprintf(str, "]");
      }
      current = parent;
    }

    if (current->next) {
      str += sprintf(str, ",");
    }
    current = current->next;
  }
}
