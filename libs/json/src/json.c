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

// Parser state tracker
typedef struct {
  const char *ptr;
  json_object_t *current;
  bool in_array;
} json_parser_t;

const int numeric_base = 10;

// Helpers
static int parse_object(json_parser_t *parser);
static int parse_value(json_parser_t *parser);
static int parse_object_name(json_parser_t *parser);
static char *parse_string(json_parser_t *parser);
static int json_strlen(const char *str);
static json_object_t *get_parent(json_object_t *object);
static bool is_empty(json_object_t *object);
static void new_child_object(json_parser_t *parser);
static void new_object(json_parser_t *parser);
static void skip_whitespace(json_parser_t *parser);
static float parse_float(json_parser_t *parser, int significand);

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

static int parse_object(json_parser_t *parser) {
  json_object_t *root = parser->current;

  while (parser->current != NULL) {
    skip_whitespace(parser);

    // Handle object
    switch (parser->ptr[0]) {
      case '\0':  // Reached EOF
        return 0;

      case '{':  // Start of object
        parser->current->type = JSON_OBJECT;
        parser->in_array = false;
        new_child_object(parser);
        parser->ptr++;
        continue;

      case '[':  // Start of array
        parser->current->type = JSON_ARRAY;
        parser->in_array = true;
        new_child_object(parser);
        parser->ptr++;
        continue;

      case '\"':  // Start of object name, or string value in array
        if (parser->in_array) {
          break;
        }
        if (parse_object_name(parser) < 0) {
          return -1;
        }
        break;

      case ',':  // Next object or value
        new_object(parser);
        parser->ptr++;
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

        parser->ptr++;

        continue;
      }

      default:  // Either we are in an array, or something is wrong.
        if (!parser->in_array) {
          printf("error: invalid JSON\r\n");
          return -1;
        }

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
  if (!root->child || root->type != JSON_OBJECT) {
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

static int parse_value(json_parser_t *parser) {
  skip_whitespace(parser);

  json_value_t value;

  // Parse value
  switch (parser->ptr[0]) {
      // Handle in json_parse()
    case '{':
    case '[':
      return 0;

    case '\"':  // String
      parser->current->type = JSON_STRING;
      value.string = parse_string(parser);
      if (!value.string) {
        return -1;
      }
      break;

    case 't':  // true
      if (strncmp(parser->ptr, "true", strlen("true")) != 0) {
        printf("error: invalid value: %4s\r\n", parser->ptr);
        return -1;
      }
      parser->current->type = JSON_BOOL;
      value.boolean = true;
      parser->ptr += strlen("true");
      break;

    case 'f':  // false
      if (strncmp(parser->ptr, "false", strlen("false")) != 0) {
        printf("error: invalid value: %5s\r\n", parser->ptr);
        return -1;
      }
      parser->current->type = JSON_BOOL;
      value.boolean = false;
      parser->ptr += strlen("false");
      break;

    case 'n':  // null
      if (strncmp(parser->ptr, "null", strlen("null")) != 0) {
        printf("error: invalid value: %4s\r\n", parser->ptr);
        return -1;
      }
      parser->current->type = JSON_NULL;
      parser->current->value = NULL;
      parser->ptr += strlen("null");
      return 0;

    default:  // Number
    {
      parser->current->type = JSON_INT;

      char *end = "";

      value.int_ = (int)strtol(parser->ptr, &end, numeric_base);

      parser->ptr = end;

      if (*end == '.' || *end == 'e' || *end == 'E') {
        parser->current->type = JSON_FLOAT;
        value.float_ = parse_float(parser, value.int_);
      }

      break;
    }
  }

  // Check end of value
  skip_whitespace(parser);
  if (parser->ptr[0] != '\0' && parser->ptr[0] != ',' &&
      parser->ptr[0] != '}' && parser->ptr[0] != ']') {
    printf("error: invalid JSON\r\n");
    return -1;
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

static int parse_object_name(json_parser_t *parser) {
  char *str = parse_string(parser);
  if (!str) {
    return -1;
  }

  parser->current->name = str;

  // Check colon and skip whitespace
  skip_whitespace(parser);
  if (parser->ptr[0] != ':') {
    printf("error: no colon after name.\r\n");
    return -1;
  }

  parser->ptr++;
  return 0;
}

static char *parse_string(json_parser_t *parser) {
  size_t str_len = json_strlen(parser->ptr);
  char *str = arena_alloc(&json_arena.arena, str_len + 1);
  if (!str) {
    printf("error: out of memory.\r\n");
    return NULL;
  }
  parser->ptr++;
  strncpy(str, parser->ptr, str_len);
  str[str_len] = '\0';
  parser->ptr += str_len + 1;  // Skip starting and closing quotes
  return str;
}

static int json_strlen(const char *str) {
  const char *ptr = str;

  ptr++;  // Skip first quote

  if (*ptr == '\"') {
    return 0;
  }

  while (++ptr) {
    if (*ptr == '\"' && *(ptr - 1) != '\\') {
      break;
    }
  }

  return (int)(ptr - str - 1);
}

static float parse_float(json_parser_t *parser, int significand) {
  char *end = "";

  // Parse fraction
  float fraction = 0;
  if (parser->ptr[0] == '.') {
    parser->ptr++;
    int decimals = 0;
    float fractional_divisor = 1;
    decimals = (int)strtol(parser->ptr, &end, numeric_base);
    while (parser->ptr[0] != *end) {
      fractional_divisor *= 10;  // NOLINT
      parser->ptr++;
    }
    fraction = (float)decimals / fractional_divisor;
    if (significand < 0) {
      fraction = -fraction;
    }
  }

  // Parse exponent
  float exponent = 0;
  float multiplier = 1;
  if (parser->ptr[0] == 'e' || parser->ptr[0] == 'E') {
    parser->ptr++;
    exponent = (float)strtol(parser->ptr, &end, numeric_base);
    parser->ptr = end;
    if (exponent < 0) {
      while (exponent < 0) {
        multiplier /= 10;  // NOLINT
        exponent++;
      }
    } else {
      while (exponent > 0) {
        multiplier *= 10;  // NOLINT
        exponent--;
      }
    }
  }

  return ((float)significand + fraction) * multiplier;
}

static bool is_empty(json_object_t *object) {
  return !object->child && !object->value && object->type != JSON_NULL;
}

static json_object_t *get_parent(json_object_t *object) {
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

static void new_child_object(json_parser_t *parser) {
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

static void new_object(json_parser_t *parser) {
  parser->current->next = arena_alloc(&json_arena.arena, sizeof(json_object_t));
  if (!parser->current->next) {
    printf("error: out of memory.\r\n");
    parser->current = NULL;
    return;
  }
  parser->current->next->prev = parser->current;
  parser->current = parser->current->next;
}

static int isspace(int token) {
  return token == '\n' || token == '\r' || token == '\t' || token == ' ';
}

static void skip_whitespace(json_parser_t *parser) {
  while (isspace(parser->ptr[0])) {
    parser->ptr++;
  }
}

static int json_sprint_value(json_object_t *current, char *str) {
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
