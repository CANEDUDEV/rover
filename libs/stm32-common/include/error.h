#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

// Simple error reporting codes for user applications
enum app_err_t {
  APP_OK = 0,
  APP_NOT_OK = -1,
};

// General error handler
void error(void);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
