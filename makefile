# uECC makefile. Partially copied and modified from https://github.com/libecc/libecc

.SUFFIXES:

# Where to put generated objects
BUILD_DIR ?= build
# Default to the previous behaviour and keep generated .o & .d files next to the source code
OBJ_DIR ?=.
include common.mk


# Static libraries to build
LIBS = $(LIBUECC)

# Compile dynamic libraries if the user asked to
ifeq ($(WITH_DYNAMIC_LIBS),1)
LIBS += $(LIBUECC_DYN)
endif

# Executables to build
TESTS_EXEC = $(BUILD_DIR)/test_compress $(BUILD_DIR)/test_compute $(BUILD_DIR)/test_ecdh $(BUILD_DIR)/test_ecdsa
# We also compile executables with dynamic linking if asked to
ifeq ($(WITH_DYNAMIC_LIBS),1)
TESTS_EXEC += $(BUILD_DIR)/test_compress_dyn $(BUILD_DIR)/test_compute_dyn $(BUILD_DIR)/test_ecdh_dyn $(BUILD_DIR)/test_ecdsa_dyn
endif

EXEC_TO_CLEAN = $(TESTS_EXEC)

# Run all tests
test: $(TESTS_EXEC)
	@for test in $(TESTS_EXEC); do \
		echo "Running $$test"; \
		$$test; \
	done

# all and clean, as you might expect
all: $(LIBS) $(TESTS_EXEC)

# Default object files extension
OBJ_FILES_EXTENSION ?= o

clean:
	@rm -f $(LIBS) $(EXEC_TO_CLEAN)
	@find $(OBJ_DIR)/ -name '*.$(OBJ_FILES_EXTENSION)' -exec rm -f '{}' \;
	@find $(OBJ_DIR)/ -name '*.d' -exec rm -f '{}' \;
	@find $(BUILD_DIR)/ -name '*.a' -exec rm -f '{}' \;
	@find $(BUILD_DIR)/ -name '*.so' -exec rm -f '{}' \;
	@find $(BUILD_DIR)/ -name '*.su' -exec rm -f '{}' \;
	@find . -name '*~'  -exec rm -f '{}' \;



# --- Source Code ---

UECC_SRC = uECC.c

# --- Static Libraries ---

LIBUECC_SRC = $(UECC_SRC)
LIBUECC_OBJECTS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(LIBUECC_SRC)))
$(LIBUECC): $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(AR) $(AR_FLAGS) $@ $^
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(RANLIB) $(RANLIB_FLAGS) $@

# --- Dynamic Libraries ---

$(LIBUECC_DYN): $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(LIB_CFLAGS) $(LIB_DYN_LDFLAGS) $^ -o $@


# --- Executables (Static linkage with libsign object files) ---

TEST_INCLUDE_DIRS = -I. -Itest

TEST_COMPRESS_SRC = test/test_compress.c
TEST_COMPRESS_OBJECTS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(TEST_COMPRESS_SRC)))
$(BUILD_DIR)/test_compress: $(TEST_COMPRESS_OBJECTS) $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) $^ -o $@

TEST_COMPUTE_SRC = test/test_compute.c
TEST_COMPUTE_OBJECTS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(TEST_COMPUTE_SRC)))
$(BUILD_DIR)/test_compute: $(TEST_COMPUTE_OBJECTS) $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) $^ -o $@

TEST_ECDH_SRC = test/test_ecdh.c
TEST_ECDH_OBJECTS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(TEST_ECDH_SRC)))
$(BUILD_DIR)/test_ecdh: $(TEST_ECDH_OBJECTS) $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) $^ -o $@

TEST_ECDSA_SRC = test/test_ecdsa.c
TEST_ECDSA_OBJECTS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(TEST_ECDSA_SRC)))
$(BUILD_DIR)/test_ecdsa: $(TEST_ECDSA_OBJECTS) $(LIBUECC_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) $^ -o $@

# --- Excutables (Dynamic linkage with libsign shared library) ---

$(BUILD_DIR)/test_compress_dyn: $(TEST_COMPRESS_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) -L$(BUILD_DIR) $^ -luecc -o $@

$(BUILD_DIR)/test_compute_dyn: $(TEST_COMPUTE_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) -L$(BUILD_DIR) $^ -luecc -o $@

$(BUILD_DIR)/test_ecdh_dyn: $(TEST_ECDH_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) -L$(BUILD_DIR) $^ -luecc -o $@

$(BUILD_DIR)/test_ecdsa_dyn: $(TEST_ECDSA_OBJECTS)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) $(BIN_CFLAGS) $(BIN_LDFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) -L$(BUILD_DIR) $^ -luecc -o $@

.PHONY: all clean 16 32 64 debug debug16 debug32 debug64 force_arch32 force_arch64

LIB_SRC = $(UECC_SRC)
LIB_OBJ = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(LIB_SRC)))

BIN_SRC = $(TEST_COMPRESS_SRC) $(TEST_COMPUTE_SRC) $(TEST_ECDH_SRC) $(TEST_ECDSA_SRC)
BIN_OBJ = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(BIN_SRC)))

# All source files, used to construct general rules
SRC += $(LIB_SRC)
SRC += $(BIN_SRC)

# All object files
OBJS = $(patsubst %,$(OBJ_DIR)/%.$(OBJ_FILES_EXTENSION),$(basename $(SRC)))

# General dependency rule between .o and .d files
DEPS = $(OBJS:.$(OBJ_FILES_EXTENSION)=.d)

# General rule for creating .o (and .d) file from .c
$(LIB_OBJ): $(LIB_SRC)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) -c $(LIB_CFLAGS) -o $@ $<

$(BIN_OBJ): $(BIN_SRC)
	$(VERBOSE_MAKE)$(CROSS_COMPILE)$(CC) -c $(BIN_CFLAGS) $(TEST_FLAGS) $(TEST_INCLUDE_DIRS) -o $@ $<

# Populate the directory structure to contain the .o and .d files, if necessary
$(shell mkdir -p $(dir $(OBJS)) >/dev/null)
$(shell mkdir -p $(BUILD_DIR) >/dev/null)

# Make a note of the MAKEFILE_LIST at this stage of parsing the Makefile
# It is important here to use the ':=' operator so it is evaluated only once,
# and to do this before all the DEPS files are included as makefiles.
MAKEFILES:=$(MAKEFILE_LIST)

# Make object files depend on all makefiles used - this forces a rebuild if any
# of the makefiles are changed
$(OBJS) : $(MAKEFILES)

# Dep files are makefiles that keep track of which header files are used by the
# c source code. Include them to allow them to work correctly.
-include $(DEPS)
