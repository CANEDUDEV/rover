#include "json.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "test.h"

char test_data[JSON_MAX_SIZE];

void setup_test(char *test_data_path);
void test_json_parse(void);
void test_get_object(void);
void test_get_nonexistent_object(void);
void test_get_nested_object(void);
void test_empty_object(void);
void test_json_insert_object(void);
void test_json_insert_nested_object(void);

int main(int argc, char **argv) {
  if (argc != 2) {
    printf("Please specify test-data.json path");
    exit(TEST_SETUP_FAIL);
  }

  setup_test(argv[1]);
  test_json_parse();
  test_get_object();
  test_get_nonexistent_object();
  test_get_nested_object();
  test_empty_object();
  test_json_insert_object();
  test_json_insert_nested_object();
}

void test_json_parse(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  char str[JSON_MAX_SIZE];
  json_sprint(json, str);

  // String generated using
  // `jq . -c libs/json/tests/test-data.json | jq . -R`
  // Plus some massaging of floats
  char *expected =
      "{\"n1\":123,\"n2\":-123,\"n3\":13.370000,\"n4\":-13.370000,\"n5\":0."
      "001337,\"n6\":-13370000.000000,\"true\":true,\"false\":false,\"null\":"
      "null,\"str\":\"abc\",\"str2\":\"xyz \\\" "
      "\",\"o1\":{},\"d1\":{\"d2\":{\"n1\":123},\"str\":\"abc\"},\"string_with_"
      "escapes\":\"\\\" \\\\ \\/ \\b \\f \\n \\r \\t "
      "\\u0000\",\"arr1\":[],\"arr2\":[true,false,null,[],1,\"a\",[1,2],{\"b\":"
      "\"c\",\"a\":[1,[[{}],2]]}]}";

  ASSERT(strcmp(str, expected) == 0, "\nexpected:\n%s\n\ngot:\n%s\n", expected,
         str);

  // Parse compacted JSON
  json = json_parse(expected);
  ASSERT(json != NULL, "couldn't parse JSON");

  json_sprint(json, str);

  ASSERT(strcmp(str, expected) == 0, "\nexpected:\n%s\n\ngot:\n%s\n", expected,
         str);
}

void test_get_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  json_object_t *object = json_get_object("n1", json);
  ASSERT(object != NULL, "couldn't find object");
  ASSERT(object->value != NULL, "object has no value");

  const int expected = 123;
  ASSERT(object->value->int_ == expected, "expected: %d, got: %d", expected,
         object->value->int_);
}

void test_get_nonexistent_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  json_object_t *object = json_get_object("none", json);
  ASSERT(object == NULL, "found nonexistent object");
}

void test_get_nested_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  json_object_t *object = json_get_object("d1", json);
  ASSERT(object != NULL, "couldn't find object");

  json_object_t *nested_object = json_get_object("str", object);
  ASSERT(nested_object != NULL, "couldn't find nested object");

  ASSERT(nested_object->value != NULL, "object has no value");

  char *expected = "abc";
  ASSERT(strcmp(nested_object->value->string, expected) == 0,
         "expected: %s, got: %s", expected, nested_object->value->string);
}

void test_empty_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");
  json_object_t *object = json_get_object("o1", json);
  ASSERT(object != NULL, "couldn't find object");
  ASSERT(object->value == NULL, "empty object has value");
  ASSERT(object->child == NULL, "empty object has child object");
}

void test_json_insert_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  const char *new_object = "\"new\": \"object\"";

  ASSERT(json_insert_object(new_object, json) == 0, "failed to insert object");

  json_object_t *object = json_get_object("new", json);

  ASSERT(object != NULL, "couldn't find object");
  ASSERT(object->value != NULL, "object has no value");

  ASSERT(strcmp(object->name, "new") == 0, "object's name is incorrect");
  ASSERT(strcmp(object->value->string, "object") == 0,
         "object's value is incorrect");

  // Check if new print includes new object
  char str[JSON_MAX_SIZE];
  json_sprint(json, str);

  char *expected =
      "{\"n1\":123,\"n2\":-123,\"n3\":13.370000,\"n4\":-13.370000,\"n5\":0."
      "001337,\"n6\":-13370000.000000,\"true\":true,\"false\":false,\"null\":"
      "null,\"str\":\"abc\",\"str2\":\"xyz \\\" "
      "\",\"o1\":{},\"d1\":{\"d2\":{\"n1\":123},\"str\":\"abc\"},\"string_with_"
      "escapes\":\"\\\" \\\\ \\/ \\b \\f \\n \\r \\t "
      "\\u0000\",\"arr1\":[],\"arr2\":[true,false,null,[],1,\"a\",[1,2],{\"b\":"
      "\"c\",\"a\":[1,[[{}],2]]}],\"new\":\"object\"}";

  ASSERT(strcmp(expected, str) == 0, "\nexpected:\n%s\n\ngot:\n%s\n", expected,
         str);
}

void test_json_insert_nested_object(void) {
  json_object_t *json = json_parse(test_data);
  ASSERT(json != NULL, "couldn't parse JSON");

  const char *new_object = "\"new\": \"object\"";

  json_object_t *object = json_get_object("o1", json);
  ASSERT(object != NULL, "couldn't find object");

  ASSERT(json_insert_object(new_object, object) == 0,
         "failed to insert object");

  object = json_get_object("new", object);

  ASSERT(object != NULL, "couldn't find object");
  ASSERT(object->value != NULL, "object has no value");

  ASSERT(strcmp(object->name, "new") == 0, "object's name is incorrect");
  ASSERT(strcmp(object->value->string, "object") == 0,
         "object's value is incorrect");

  // Check if new print includes new object
  char str[JSON_MAX_SIZE];
  json_sprint(json, str);

  char *expected =
      "{\"n1\":123,\"n2\":-123,\"n3\":13.370000,\"n4\":-13.370000,\"n5\":0."
      "001337,\"n6\":-13370000.000000,\"true\":true,\"false\":false,\"null\":"
      "null,\"str\":\"abc\",\"str2\":\"xyz \\\" "
      "\",\"o1\":{\"new\":\"object\"},\"d1\":{\"d2\":{\"n1\":123},\"str\":"
      "\"abc\"},\"string_with_escapes\":\"\\\" \\\\ \\/ \\b \\f \\n \\r \\t "
      "\\u0000\",\"arr1\":[],\"arr2\":[true,false,null,[],1,\"a\",[1,2],{\"b\":"
      "\"c\",\"a\":[1,[[{}],2]]}]}";

  ASSERT(strcmp(expected, str) == 0, "\nexpected:\n%s\n\ngot:\n%s\n", expected,
         str);
}

void setup_test(char *test_data_path) {
  // Open the JSON file in read mode
  FILE *file = fopen(test_data_path, "r");
  if (!file) {
    printf("Could not open file\n");
    exit(TEST_SETUP_FAIL);
  }

  // Read the file into the fixed-size buffer
  size_t bytes_read = fread(test_data, 1, sizeof(test_data) - 1, file);
  test_data[bytes_read] = '\0';  // Null-terminate the string

  // Close the file
  fclose(file);
}
