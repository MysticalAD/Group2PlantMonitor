#include <pgmspace.h>

#define SECRET
// Amazon Root CA 1, ai-camps aws account, VeriSign-Class 3-Public-Primary-Certification-Authority-G5
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIE0zCCA7ugAwIBAgIQGNrRniZ96LtKIVjNzGs7SjANBgkqhkiG9w0BAQUFADCB
yjELMAkGA1UEBhMCVVMxFzAVBgNVBAoTDlZlcmlTaWduLCBJbmMuMR8wHQYDVQQL
ExZWZXJpU2lnbiBUcnVzdCBOZXR3b3JrMTowOAYDVQQLEzEoYykgMjAwNiBWZXJp
U2lnbiwgSW5jLiAtIEZvciBhdXRob3JpemVkIHVzZSBvbmx5MUUwQwYDVQQDEzxW
ZXJpU2lnbiBDbGFzcyAzIFB1YmxpYyBQcmltYXJ5IENlcnRpZmljYXRpb24gQXV0
aG9yaXR5IC0gRzUwHhcNMDYxMTA4MDAwMDAwWhcNMzYwNzE2MjM1OTU5WjCByjEL
MAkGA1UEBhMCVVMxFzAVBgNVBAoTDlZlcmlTaWduLCBJbmMuMR8wHQYDVQQLExZW
ZXJpU2lnbiBUcnVzdCBOZXR3b3JrMTowOAYDVQQLEzEoYykgMjAwNiBWZXJpU2ln
biwgSW5jLiAtIEZvciBhdXRob3JpemVkIHVzZSBvbmx5MUUwQwYDVQQDEzxWZXJp
U2lnbiBDbGFzcyAzIFB1YmxpYyBQcmltYXJ5IENlcnRpZmljYXRpb24gQXV0aG9y
aXR5IC0gRzUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCvJAgIKXo1
nmAMqudLO07cfLw8RRy7K+D+KQL5VwijZIUVJ/XxrcgxiV0i6CqqpkKzj/i5Vbex
t0uz/o9+B1fs70PbZmIVYc9gDaTY3vjgw2IIPVQT60nKWVSFJuUrjxuf6/WhkcIz
SdhDY2pSS9KP6HBRTdGJaXvHcPaz3BJ023tdS1bTlr8Vd6Gw9KIl8q8ckmcY5fQG
BO+QueQA5N06tRn/Arr0PO7gi+s3i+z016zy9vA9r911kTMZHRxAy3QkGSGT2RT+
rCpSx4/VBEnkjWNHiDxpg8v+R70rfk/Fla4OndTRQ8Bnc+MUCH7lP59zuDMKz10/
NIeWiu5T6CUVAgMBAAGjgbIwga8wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8E
BAMCAQYwbQYIKwYBBQUHAQwEYTBfoV2gWzBZMFcwVRYJaW1hZ2UvZ2lmMCEwHzAH
BgUrDgMCGgQUj+XTGoasjY5rw8+AatRIGCx7GS4wJRYjaHR0cDovL2xvZ28udmVy
aXNpZ24uY29tL3ZzbG9nby5naWYwHQYDVR0OBBYEFH/TZafC3ey78DAJ80M5+gKv
MzEzMA0GCSqGSIb3DQEBBQUAA4IBAQCTJEowX2LP2BqYLz3q3JktvXf2pXkiOOzE
p6B4Eq1iDkVwZMXnl2YtmAl+X6/WzChl8gGqCBpH3vn5fJJaCGkgDdk+bW48DW7Y
5gaRQBi5+MHt39tBquCWIMnNZBU4gcmU7qKEKQsTb47bDN0lAtukixlE0kF6BWlK
WE9gyn6CagsCqiUXObXbf+eEZSqVir2G3l6BFoMtEMze/aiCKm0oHw0LxOXnGiYZ
4fQRbxC1lfznQgUy286dUV4otp6F01vvpX1FQHKOtw5rDgb7MzVIcbidJ4vEZV8N
hnacRHr2lVz2XTIIM6RUthg/aFzyQkqFOFSDX9HoLPKsEdao7WNq
-----END CERTIFICATE-----
)EOF";

// Device Certificate Pem                                               
static const char AWS_CERT_CRT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUZl2ZstBqFURAPrwzSFsd1G2IF4wwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MDMyODE5MTM0
NVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALX2/vnPn+V9ejzHlktp
1oU6EQOdtsk5sRxhDAzK94mCWOCbzt68f8pSZtiS4NgUFwW0yXi1G9S7Mi2LnB7l
6m/ldJI0jJL5PWplsvlLAzFC+p3B58suxsPBce44EES2+cvF2pcquyvwMm1hdb7t
9wBqp7bcdsXs/hfP9RMkcpq38+PtfBjWWEE1abJd9HN9HMNtnSU6rgLLxLZQpzy/
6BR5GIMNp7wZFdloiOb5+rIK5k6tPHMVCsIZXYTJLQuoyE0mXcQjWu9aaDvF1d5e
7g8G183NDK/aRyAP9nd1tO0U4OPtbnVQImSTElZTvqwYGHxWmbP2IYNfXNXiDfvP
wekCAwEAAaNgMF4wHwYDVR0jBBgwFoAUNG+12Md2diwb8YxBZdLIGMQmpT0wHQYD
VR0OBBYEFKqytrOHRsD5WBV2YRimlybCBHfnMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBlyWDzL4tpl7TDgJZX14elWz93
Ex+vpx2pcz7giSjafVjNRQ/KbXo++piyL9+Rjphsx7wddahAKZM0fznsHPvYdaPn
U8ADDQOtgoSt+9XXzEH73sW9mVPIz4shCtMEKr20/SMW0C6cvKhWT7P2fOnXptS1
MeK8M2xX447UTDLrU7IKZ3uLIBBkrpcdZ4IUKGk4SK9uYOZ4M70eBRrTvgN5gjvt
PI/oq5VlniZuiSqqh5DPsCkSR23Bxq3kvrQF9vBlRWeSN+KeedQzRNIP5Xxcia/m
ztWKoDIXZdn5JTVeI7a6+noskXFoUkVs2emNLswOeEqf16svHjm+T5EH5qH+
-----END CERTIFICATE-----
)EOF";

// Device Private Pem                                         
static const char AWS_PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEAtfb++c+f5X16PMeWS2nWhToRA522yTmxHGEMDMr3iYJY4JvO
3rx/ylJm2JLg2BQXBbTJeLUb1LsyLYucHuXqb+V0kjSMkvk9amWy+UsDMUL6ncHn
yy7Gw8Fx7jgQRLb5y8Xalyq7K/AybWF1vu33AGqnttx2xez+F8/1EyRymrfz4+18
GNZYQTVpsl30c30cw22dJTquAsvEtlCnPL/oFHkYgw2nvBkV2WiI5vn6sgrmTq08
cxUKwhldhMktC6jITSZdxCNa71poO8XV3l7uDwbXzc0Mr9pHIA/2d3W07RTg4+1u
dVAiZJMSVlO+rBgYfFaZs/Yhg19c1eIN+8/B6QIDAQABAoIBAC5K/UlOog4/YvbN
jBX5giL4x9VUrWj6LW3b3rXueRVZvQS/fty5tgUh+5pDm8I9MjI4cT0tTjxLt5NQ
vlI6Ce+Xv5VmmxDVKePEpci4ZcKFmL7D0xWk+9pXf4Wsa68foeNOnbLxoxalkB61
ub/uSHWprKtf4TOvfGFQfbf2ANYNYhYZLCFRDiwIUuu2yMoXo7Yw6g/2C3py0l24
ADyRGe299PWrxZkr3xycwH51AlY6r4s8r7yJ7o9eMwVxTB0A/EneeZCJ1pRZwK2x
Uzys2AG1Y5RUqs9s2l6NaHaA6CNR2a1NzfVRAIN0C7HsDQYnVIGCJeWUbrBopMM7
Am4OF1UCgYEA8Tq2jfhNWo10PItV+409hE6pt5qxcpukEnZ0vbZN755ghz0AnmPa
BUWz8RdQHsw+lEb+4Wu+9o18aLGSbJj/Imn9Wk54GDCo5jl2YaglzwVwIMQcT1nL
iUnfwf77y3pcOA3FNWTbzeiKvZy2Gq0BKH26CbdzkeaU0V9ANTG55p8CgYEAwRtO
seaKIpTeqDcsQwQPQ30fpjUG7gqEzonz3YpLPV+K3lOY8ISU+3kRbDO/5TbGkOv+
dWtEKLq8Rdq/dOqH+MlmSQtliSAgNf/C8vXfYYsQeBcPYWjLemUbuObSKGWQH+Pw
EZY8Yc4Pru5oRTZISn6YBtjeMMWwiD2v5q8VsncCgYBepBpbOL+gdz7TQhdJyfAI
qJQNZLOZs1Z7p93FUHDjQc+9P0Gu4p3205H1VKAqdRdYNUJqTkttj7TvnjtEQAJZ
DhxIAeE8HFsneAPp7H5o6klDka1JtIEWK9WN1uwSuL7VfxGQETHrZwCHw6Puc1Gx
ASKsDT1aUIyyWBxowkmdIQKBgGNDPjiEYQ1VqP6fHcl1FMOEVfiB2YYTMiFZGFoQ
9IwfyKpwHnXEMTDzEtmj4Eha18c3CB4jfW3ST2U5BKF8zVg9bq6thQvvz57yUavC
Z4kkitqwP9+6lp3by1xIik7ppiQ+bvEbYP6FhdpOAptRQ+fHGy6p280GUbA2xWy3
I1gRAoGAcgRhShSkXWAOnZ4qsOPpTMoZCThCVMCfUEnH4MbXWBBPOcY0FpU4Weop
P7pmVKZjrvcxvzVdSFB6o4zGFN4UJItRkBmbJSriPFdV/yv1v8204i7mRwAnCrg0
ySuWpSvSuLaGsI67Wa+JfN9v0iSfTrAikzp9UErCW/KuxPo55YA=
-----END RSA PRIVATE KEY-----
)EOF";
