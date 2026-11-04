#pragma once

// FreeInk SDK — TLS 1.3 secure client.
//
// WHY: the precompiled mbedTLS shipped in the ESP-IDF/pioarduino package has
// TLS 1.3 compiled out as empty stubs (PSA crypto prerequisites disabled), so
// WiFiClientSecure / esp_http_client cannot reach TLS-1.3-only servers
// (e.g. KOSync at kosync.ak-team.com:3042 — handshake fails with
// -0x7780 MBEDTLS_ERR_SSL_FATAL_ALERT_MESSAGE). A -D Kconfig flag can't change a
// precompiled .a, and a custom_sdkconfig rebuild fails on managed-component
// dependencies. The only fix that doesn't rebuild ESP-IDF is to bring our own
// TLS stack compiled from source: wolfSSL, which supports TLS 1.3 + PSA.
//
// SecureClient is an Arduino Client wrapping a wolfSSL session over a plain
// WiFiClient transport, independent of system mbedTLS.
//
// OPT-IN: enable with -DFREEINK_NET_WOLFSSL=1 and add wolfSSL to lib_deps. With
// the flag off, this compiles to an inert no-op (connectSecure() returns false)
// so the rest of the SDK builds without the wolfSSL dependency present.

#include <Arduino.h>
#include <Client.h>
#include <WiFiClient.h>

namespace freeink {

class SecureClient : public Client {
 public:
  SecureClient() = default;
  ~SecureClient() override;

  // Lend wolfSSL one pre-allocated full-record buffer for the length of a bulk
  // download, then take it back.
  //
  // WHY: a server that ignores max_fragment_length sends 16 KB TLS records.
  // wolfSSL releases its grown input buffer after each record and asks for a
  // fresh ~16.4 KB block for the next one, so a long download needs that block
  // over and over. On a board with no PSRAM the heap is fragmented long before
  // the transfer starts -- measured on an Xteink X3, 2026-11-04: 108 kB free on
  // the home screen but a largest contiguous block of only 30.7 kB, and 27.6 kB
  // once WiFi is up. A 4 MB firmware download died 61,810 bytes in with
  // wolfSSL MEMORY_E.
  //
  // So: take the block ONCE, while a large one still exists, and hand it back
  // to wolfSSL each time it asks for a record-sized allocation. The idea is
  // upstream CrossPoint's (freeink-sdk 118c6723 and its follow-up 813a3c82);
  // this is our own implementation of it against our pinned SDK.
  //
  // Call reserve() AFTER the request has been written, not right after the
  // handshake: the request string is freed by then, and taking the big block
  // first can make building that string fail instead -- std::string has no
  // failure path under -fno-exceptions. That ordering is upstream's lesson,
  // learned the expensive way.
  //
  // Returns false when the block could not be taken; the caller carries on
  // unreserved, which is exactly today's behaviour. Safe to call when wolfSSL
  // is compiled out: it does nothing and returns false.
  static bool reserveRecordBuffer();
  // Idempotent. If wolfSSL is still holding the buffer the free is deferred
  // until it hands it back, so this can be called from a teardown path without
  // knowing what the TLS layer is doing.
  static void releaseRecordBuffer();

  // Certificate / verification configuration (applied before connect()).
  void setCACert(const char* rootCA);
  void setInsecure();  // skip peer verification (testing only)

  // Connect and perform a TLS 1.3 handshake to host:port (uses the SNI host).
  int connect(IPAddress ip, uint16_t port) override;
  int connect(const char* host, uint16_t port) override;

  size_t write(uint8_t b) override;
  size_t write(const uint8_t* buf, size_t size) override;
  int available() override;
  int read() override;
  int read(uint8_t* buf, size_t size) override;
  int peek() override;
  void flush() override;
  void stop() override;
  uint8_t connected() override;
  operator bool() override { return connected(); }

  // True if the library was built with wolfSSL TLS 1.3 support enabled.
  static bool tls13Available();

 private:
  int connectWithMethod(const char* host, uint16_t port, void* method, const char* label);

  WiFiClient _transport;
  const char* _rootCA = nullptr;
  bool _insecure = false;
  void* _ssl = nullptr;  // WOLFSSL* (opaque to keep wolfSSL headers out of here)
  void* _ctx = nullptr;  // WOLFSSL_CTX*
  bool _connected = false;
};

}  // namespace freeink
