#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

// Simple error reporting codes for user applications
#define APP_OK 0
#define APP_NOT_OK 1

// General error handler
void error(void);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
