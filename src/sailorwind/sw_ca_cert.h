#pragma once
//
// sw_ca_cert.h — root CA for sailorwind.net TLS (ISRG Root X1, Let's Encrypt).
//
// The sailorwind submitter + provisioner pin this root the same way
// ota_update_task.cpp does (sailorwind.net serves under a Let's Encrypt chain
// anchored at ISRG Root X1). Kept as a separate, sailorwind-owned copy so the
// additive module never has to reach into the upstream OTA translation unit
// (whose copy is file-static). If Let's Encrypt rotates roots, update both.

namespace sailorwind {

// PEM, NUL-terminated. Pass to WiFiClientSecure::setCACert().
extern const char* const kSwIsrgRootX1Pem;

}  // namespace sailorwind
