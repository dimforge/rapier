#ifndef EXAMPLE_FILES_H
#define EXAMPLE_FILES_H

/* Directory enumeration shared by the example's scene-discovery code. */
typedef struct FileNames {
    char **names;
    size_t count;
} FileNames;

static void addFilename(FileNames *files, const char *name) {
    char **names = realloc(files->names, (files->count + 1) * sizeof(*names));
    if (!names) {
        abort();
    }
    files->names = names;
    names[files->count] = malloc(strlen(name) + 1);
    if (!names[files->count]) {
        abort();
    }
    strcpy(names[files->count++], name);
}

static void freeFilenames(FileNames *files) {
    for (size_t i = 0; i < files->count; ++i) {
        free(files->names[i]);
    }
    free(files->names);
    *files = (FileNames){0};
}
#ifdef _WIN32
#include <windows.h>

static FileNames listDirectory(const char *path) {
    FileNames result = {0};
    char pattern[8192];
    snprintf(pattern, sizeof(pattern), "%s/*", path);
    WIN32_FIND_DATAA entry;
    HANDLE directory = FindFirstFileA(pattern, &entry);
    if (directory == INVALID_HANDLE_VALUE) {
        return result;
    }
    do {
        if (strcmp(entry.cFileName, ".") && strcmp(entry.cFileName, "..")) {
            addFilename(&result, entry.cFileName);
        }
    } while (FindNextFileA(directory, &entry));
    FindClose(directory);
    return result;
}
#else
#include <dirent.h>

static FileNames listDirectory(const char *path) {
    FileNames result = {0};
    DIR *directory = opendir(path);
    if (!directory) {
        return result;
    }
    struct dirent *entry;
    while ((entry = readdir(directory))) {
        if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) {
            addFilename(&result, entry->d_name);
        }
    }
    closedir(directory);
    return result;
}
#endif
#endif
