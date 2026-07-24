// test_sha256.c — verifies the trimmed sha256.h core against known NIST
// FIPS 180-4 test vectors before it's trusted for .gband content-addressing.
#include "../src/sha256.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void hex_encode(const unsigned char *in, int len, char *out) {
    static const char hexchars[] = "0123456789abcdef";
    for (int i = 0; i < len; i++) {
        out[i * 2] = hexchars[in[i] >> 4];
        out[i * 2 + 1] = hexchars[in[i] & 0xF];
    }
    out[len * 2] = '\0';
}

static void check(const char *label, const unsigned char *input, size_t len, const char *expected_hex) {
    unsigned char digest[32];
    sha256(input, len, digest);
    char got_hex[65];
    hex_encode(digest, 32, got_hex);
    if (strcmp(got_hex, expected_hex) == 0) {
        printf("PASS: %s\n", label);
    } else {
        printf("FAIL: %s -- got %s, want %s\n", label, got_hex, expected_hex);
        failures++;
    }
}

int main(void) {
    // NIST FIPS 180-4 test vectors, verified independently via Python's
    // hashlib before being hardcoded here.
    check("sha256(\"\")", (const unsigned char *)"", 0,
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    check("sha256(\"abc\")", (const unsigned char *)"abc", 3,
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    check("sha256(two-block 56-byte message)",
          (const unsigned char *)"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56,
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    if (failures == 0) {
        printf("\nALL PASS\n");
    } else {
        printf("\n%d FAILURE(S)\n", failures);
    }
    return failures ? 1 : 0;
}
