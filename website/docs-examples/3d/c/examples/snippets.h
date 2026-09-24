/* Shared by the user-guide snippets: any failing Rapier call aborts the snippet (and fails its test). */
#ifndef SNIPPETS_H
#define SNIPPETS_H
#include "rapier.h"
#include "rapier_helpers.h"
#include "rapier_math.h"
#include <stdio.h>
#include <stdlib.h>

static void RAPIER_CALL snippets_abort_on_error(R3Status status, const char *message, void *user_data) {
    (void)user_data;
    fprintf(stderr, "Rapier error %u: %s\n", (unsigned)status, message);
    exit(EXIT_FAILURE);
}

/* Call at the start of main, so that any failing Rapier call aborts the snippet. */
static inline void snippets_init(void) {
    R3ErrorHandler handler = {snippets_abort_on_error, NULL};
    r3SetErrorHandler(handler);
}
#endif
