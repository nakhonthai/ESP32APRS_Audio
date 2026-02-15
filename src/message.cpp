

#include <Arduino.h>
#include <handleATCommand.h>
#include "message.h"
#include <parse_aprs.h>
#include "webservice.h"
#include <MD5Builder.h>

#define AES_BLOCK_SIZE 16

RTC_DATA_ATTR uint16_t msgID = 0;

msgType *msgQueue;

extern Configuration config;
extern bool psramBusy;

// แปลง bytes → HEX string
String bytesToHexString(const uint8_t *data, size_t len)
{
    String hexString = "";
    const char hexChars[] = "0123456789ABCDEF";

    for (size_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        hexString += hexChars[(b >> 4) & 0x0F];
        hexString += hexChars[b & 0x0F];
    }

    return hexString;
}

// แปลง HEX string → bytes
size_t hexStringToBytes(const String &hexString, uint8_t *output, size_t maxLen)
{
    size_t len = hexString.length();
    size_t bytesCount = 0;

    // ตรวจสอบความยาวต้องเป็นเลขคู่
    if (len % 2 != 0)
        return 0;

    for (size_t i = 0; i < len; i += 2)
    {
        if (bytesCount >= maxLen)
            break;

        char c1 = hexString[i];
        char c2 = hexString[i + 1];

        uint8_t high = (c1 >= '0' && c1 <= '9') ? c1 - '0' : (c1 >= 'A' && c1 <= 'F') ? c1 - 'A' + 10
                                                         : (c1 >= 'a' && c1 <= 'f')   ? c1 - 'a' + 10
                                                                                      : 0;

        uint8_t low = (c2 >= '0' && c2 <= '9') ? c2 - '0' : (c2 >= 'A' && c2 <= 'F') ? c2 - 'A' + 10
                                                        : (c2 >= 'a' && c2 <= 'f')   ? c2 - 'a' + 10
                                                                                     : 0;

        output[bytesCount++] = (high << 4) | low;
    }

    return bytesCount;
}

// ---------- PKCS7 Padding ----------
static void pkcs7_pad(const uint8_t *input, size_t in_len, uint8_t **out, size_t *out_len)
{
    size_t pad = AES_BLOCK_SIZE - (in_len % AES_BLOCK_SIZE);
    *out_len = in_len + pad;
    *out = (uint8_t *)malloc(*out_len);
    memcpy(*out, input, in_len);
    memset(*out + in_len, (uint8_t)pad, pad);
}

static bool pkcs7_unpad(uint8_t *buf, size_t buf_len, size_t *out_len)
{
    if (buf_len == 0 || buf_len % AES_BLOCK_SIZE != 0)
        return false;
    uint8_t pad = buf[buf_len - 1];
    if (pad == 0 || pad > AES_BLOCK_SIZE)
        return false;
    for (size_t i = 0; i < pad; ++i)
    {
        if (buf[buf_len - 1 - i] != pad)
            return false;
    }
    *out_len = buf_len - pad;
    return true;
}

// ---------- AES Encrypt (auto IV prepend + Base64) ----------
String aesEncryptBase64WithIV(const String &plain, const uint8_t key[16], uint16_t msgID)
{
    size_t in_len = plain.length();
    const uint8_t *in_ptr = (const uint8_t *)plain.c_str();

    // 1. Generate random IV
    uint8_t iv[AES_BLOCK_SIZE];
    // esp_fill_random(iv, 8);
    // time_t timeStamp = time(NULL) / 60;

    // char input[20];
    // sprintf(input,"%s%d",config.msg_mycall,timeStamp);
    String input = String(config.msg_mycall);
    input.trim();
    input += "_" + String(msgID);
    log_d("input = %s", input.c_str());

    // Generate MD5 hash
    MD5Builder md5;
    md5.begin();
    md5.add(input);  // Add the input data
    md5.calculate(); // Compute the MD5 hash
    md5.getBytes(iv);

    log_d("Encrypt IV %s", bytesToHexString(iv, 16).c_str());

    // 2. PKCS7 pad
    uint8_t *padded = nullptr;
    size_t padded_len = 0;
    pkcs7_pad(in_ptr, in_len, &padded, &padded_len);

    // 3. AES CBC encrypt
    uint8_t *cipher = (uint8_t *)malloc(padded_len);
    if (!cipher)
    {
        free(padded);
        return String();
    }

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 128);

    uint8_t iv_copy[AES_BLOCK_SIZE];
    memcpy(iv_copy, iv, AES_BLOCK_SIZE);

    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, padded_len, iv_copy, padded, cipher);
    mbedtls_aes_free(&aes);

    // 4. Combine IV + ciphertext
    size_t total_len = 0 + padded_len;
    uint8_t *combined = (uint8_t *)malloc(total_len);
    memcpy(combined, iv, 0);
    memcpy(combined + 0, cipher, padded_len);

    // 5. Base64 encode
    size_t olen = 0;
    size_t b64_max = 4 * ((total_len + 2) / 3) + 4;
    uint8_t *b64_buf = (uint8_t *)malloc(b64_max);
    mbedtls_base64_encode(b64_buf, b64_max, &olen, combined, total_len);

    String result = String((char *)b64_buf, olen);

    free(padded);
    free(cipher);
    free(combined);
    free(b64_buf);
    return result;
}

// ---------- AES Decrypt (auto extract IV from Base64) ----------
String aesDecryptBase64WithIV(const String &b64, const uint8_t key[16], const char *callsign, uint16_t msgID)
{
    size_t b64_len = b64.length();
    const uint8_t *b64_ptr = (const uint8_t *)b64.c_str();

    // 1. Decode Base64
    size_t dec_max = (b64_len / 4) * 3 + 4;
    uint8_t *decoded = (uint8_t *)malloc(dec_max);
    size_t dec_len = 0;
    if (mbedtls_base64_decode(decoded, dec_max, &dec_len, b64_ptr, b64_len) != 0)
    {
        free(decoded);
        return String();
    }

    if (dec_len <= AES_BLOCK_SIZE / 2)
    {
        free(decoded);
        return String();
    }

    // 2. Extract IV + ciphertext
    uint8_t iv[AES_BLOCK_SIZE];
    // memcpy(iv, decoded,8);

    // time_t timeStamp = time(NULL) / 60;
    String input = String(callsign);
    input.trim();
    input += "_" + String(msgID);

    log_d("input = %s", input.c_str());

    // Generate MD5 hash
    MD5Builder md5;
    md5.begin();
    md5.add(input);  // Add the input data
    md5.calculate(); // Compute the MD5 hash
    md5.getBytes(iv);

    log_d("Decrypt IV %s", bytesToHexString(iv, 16).c_str());

    size_t cipher_len = dec_len - 0;
    uint8_t *cipher = decoded + 0;

    // 3. AES decrypt
    uint8_t *plain_padded = (uint8_t *)malloc(cipher_len);
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, key, 128);

    uint8_t iv_copy[AES_BLOCK_SIZE];
    memcpy(iv_copy, iv, AES_BLOCK_SIZE);

    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, cipher_len, iv_copy, cipher, plain_padded);
    mbedtls_aes_free(&aes);

    // 4. Unpad
    size_t out_len = 0;
    if (!pkcs7_unpad(plain_padded, cipher_len, &out_len))
    {
        free(decoded);
        free(plain_padded);
        return String();
    }

    String result = String((char *)plain_padded, out_len);

    free(decoded);
    free(plain_padded);
    return result;
}

void pkgMsgSort(msgType a[])
{
    msgType t;
    char *ptr1;
    char *ptr2;
    char *ptr3;
    ptr1 = (char *)&t;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    for (int i = 0; i < (PKGLISTSIZE - 1); i++)
    {
        for (int o = 0; o < (PKGLISTSIZE - (i + 1)); o++)
        {
            if (a[o].time > a[o + 1].time)
            {
                ptr2 = (char *)&a[o];
                ptr3 = (char *)&a[o + 1];
                memcpy(ptr1, ptr2, sizeof(msgType));
                memcpy(ptr2, ptr3, sizeof(msgType));
                memcpy(ptr3, ptr1, sizeof(msgType));
            }
        }
    }
    psramBusy = false;
}

int pkgMsg_Find(const char *call, uint16_t msgID, bool rxtx)
{
    int i;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if ((strstr(msgQueue[(int)i].callsign, call) != NULL) && (msgQueue[(int)i].msgID == msgID) && (msgQueue[(int)i].rxtx == rxtx))
            return i;
    }
    return -1;
}

int pkgMsgOld()
{
    int i, ret = -1;
    time_t minimum = time(NULL) + 86400; // pkgList[0].time;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (msgQueue[(int)i].time < minimum)
        {
            minimum = msgQueue[(int)i].time;
            ret = i;
        }
    }
    return ret;
}

msgType getMsgList(int idx)
{
    msgType ret;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    memset(&ret, 0, sizeof(msgType));
    if (idx < PKGLISTSIZE)
        memcpy(&ret, &msgQueue[idx], sizeof(msgType));
    psramBusy = false;
    return ret;
}

int pkgMsgUpdate(const char *call, const char *raw, uint16_t msg_id, int8_t ack, bool rxtx)
{
    size_t len;
    if (*call == 0)
        return -1;
    if (*raw == 0)
        return -1;

    // int start_info = strchr(':',0);

    char callsign[11];
    size_t sz = strlen(call);
    memset(callsign, 0, 11);
    if (sz > 10)
        sz = 10;
    // strncpy(callsign, call, sz);
    memcpy(callsign, call, sz);

#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    int i = -1;
    // if (ack > 0) // Check ACK to update
    //{
    i = pkgMsg_Find(call, msg_id, rxtx);
    //}

    if (i < 0)
    {
        i = pkgMsgOld(); // Search free in array
        if (i > PKGLISTSIZE || i < 0)
        {
            psramBusy = false;
            return -1;
        }
    }

    msgQueue[i].time = time(NULL);
    msgQueue[i].msgID = msg_id;
    msgQueue[i].ack = ack;
    msgQueue[i].rxtx = rxtx;

    // strcpy(pkgList[i].calsign, callsign);
    memset(msgQueue[i].callsign, 0, sizeof(msgQueue[i].callsign));
    memcpy(msgQueue[i].callsign, callsign, strlen(callsign));
    len = strlen(raw);
    msgQueue[i].length = len + 1;
    if (msgQueue[i].text != NULL)
    {
        msgQueue[i].text = (char *)realloc(msgQueue[i].text, msgQueue[i].length);
        log_d("Realloc: msgQueue[%d] callsign:%s msgID:%d ack:%i", i, callsign, msg_id, ack);
    }
    else
    {
        msgQueue[i].text = (char *)calloc(msgQueue[i].length, sizeof(char));
        log_d("Calloc: msgQueue[%d] callsign:%s msgID:%d ack:%i", i, callsign, msg_id, ack);
    }
    if (msgQueue[i].text)
    {
        memset(msgQueue[i].text, 0, msgQueue[i].length);
        memcpy(msgQueue[i].text, raw, len);
        //msgQueue[i].text[len] = 0;
        log_d("New: msgQueue[%d] callsign:%s msgID:%d ack:%i", i, msgQueue[i].callsign, msgQueue[i].msgID, msgQueue[i].ack);
    }
    //}
    psramBusy = false;
    return i;
}

// ===== ส่งข้อความ APRS =====
void sendAPRSMessage(const String &toCall, const String &message, bool encrypt)
{
    ++msgID;
    if (toCall == "")
        return;
    if (message == "")
        return;

    // Normalize FROM callsign (mycall)
    String myCallUP = String(config.msg_mycall);
    myCallUP.trim();
    myCallUP.toUpperCase();

    // Normalize TO callsign
    String toCallUP = toCall;
    toCallUP.trim();
    toCallUP.toUpperCase();

    // ฟอร์แมต: FROM>APRS,TCPIP*::TOCALL   :MESSAGE
    char toCallFixed[10];
    memset(toCallFixed, 0x20, 10);

    size_t n = toCallUP.length();
    if (n > 9)
        n = 9;

    memcpy(toCallFixed, toCallUP.c_str(), n);
    toCallFixed[9] = 0;
    // snprintf(toCallFixed, sizeof(toCallFixed), "%-9s", toCall.c_str()); // 9 char padded

    String encrypted = "";
    if (encrypt)
    {
        uint8_t aes_key[16];
        hexStringToBytes(config.msg_key, aes_key, sizeof(aes_key));
        encrypted = aesEncryptBase64WithIV(message, aes_key, msgID);
    }
    else
    {
        encrypted = message;
    }
    String path = "";
    if (config.msg_path < 5)
    {
        if (config.msg_path > 0)
            path += "-" + String(config.msg_path);
    }
    else
    {
        path += ",";
        path += getPath(config.msg_path);
    }

    String packet = myCallUP + ">APE32L" + path + "::" + String(toCallFixed) + ":" + encrypted + "{" + String(msgID);

    uint8_t SendMode = 0;
    if (config.msg_rf)
        SendMode |= RF_CHANNEL;
    if (config.msg_inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(packet.c_str(), packet.length(), 0, SendMode);
    log_d("Send APRS Message to %s msgID %d TNC2: %s", toCall.c_str(), msgID, packet.c_str());
    if (config.msg_retry == 0)
        pkgMsgUpdate(toCall.c_str(), message.c_str(), msgID, -2, false); // -2=No retry
    else
        pkgMsgUpdate(toCall.c_str(), message.c_str(), msgID, config.msg_retry, false);
    event_chatMessage(false);
    // log_d(">> " + packet);
}

void sendAPRSMessageRetry()
{
    time_t timeStamp = time(NULL);
    for (int i = 0; i < PKGLISTSIZE; i++)
    {
        if ((msgQueue[i].ack > 0) && (timeStamp - msgQueue[i].time) > config.msg_interval)
        {
            if(--msgQueue[i].ack>0) msgQueue[i].time = timeStamp+config.msg_interval; // update time only if retry left

            // ฟอร์แมต: FROM>APRS,TCPIP*::TOCALL   :MESSAGE
            char toCallFixed[10];
            memset(toCallFixed, 0x20, 10);
            memcpy(toCallFixed, msgQueue[i].callsign, strlen(msgQueue[i].callsign));
            toCallFixed[9] = 0;
            // snprintf(toCallFixed, sizeof(toCallFixed), "%-9s", toCall.c_str()); // 9 char padded

            String encrypted = "";
            if (config.msg_encrypt)
            {
                uint8_t aes_key[16];
                hexStringToBytes(config.msg_key, aes_key, sizeof(aes_key));
                encrypted = aesEncryptBase64WithIV(String(msgQueue[i].text), aes_key, msgQueue[i].msgID);
            }
            else
            {
                encrypted = String(msgQueue[i].text);
            }
            String path = "";
            if (config.msg_path < 5)
            {
                if (config.msg_path > 0)
                    path += "-" + String(config.msg_path);
            }
            else
            {
                path += ",";
                path += getPath(config.msg_path);
            }
            String packet = String(config.msg_mycall) + ">APE32L" + path + "::" + String(toCallFixed) + ":" + encrypted + "{" + String(msgQueue[i].msgID);
            uint8_t SendMode = 0;
            if (config.msg_rf)
                SendMode |= RF_CHANNEL;
            if (config.msg_inet)
                SendMode |= INET_CHANNEL;
            pkgTxPush(packet.c_str(), packet.length(), 0, SendMode);
            log_d("Retry APRS Message msgQueue[%i] to %s msgID %d ack left %i TNC2: %s", i, msgQueue[i].callsign, msgQueue[i].msgID, msgQueue[i].ack, packet.c_str());
            //pkgMsgUpdate(msgQueue[i].callsign, msgQueue[i].text, msgQueue[i].msgID, msgQueue[i].ack);
            // log_d(">> " + packet);
            event_chatMessage(false);
        }
    }
}

void sendAPRSAck(const String &toCall, const String &msgNo)
{
    // ฟอร์แมต ACK: :TOCALL   :ackNN
    char toCallFixed[10];
    memset(toCallFixed, 0x20, 10);
    memcpy(toCallFixed, toCall.c_str(), toCall.length());
    toCallFixed[9] = 0;
    // snprintf(toCallFixed, sizeof(toCallFixed), "%-9s", toCall.c_str());
    String path = "";
    if (config.msg_path < 5)
    {
        if (config.msg_path > 0)
            path += "-" + String(config.msg_path);
    }
    else
    {
        path += ",";
        path += getPath(config.msg_path);
    }

    String packet = String(config.msg_mycall) + ">APE32L" + path + "::" + String(toCallFixed) + ":ack" + msgNo;
    uint8_t SendMode = 0;
    if (config.msg_rf)
        SendMode |= RF_CHANNEL;
    if (config.msg_inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(packet.c_str(), packet.length(), 0, SendMode);
    log_d("Send APRS ACK to %s msgNo %s TNC2: %s", toCall.c_str(), msgNo.c_str(), packet.c_str());
}

// ===== จัดการข้อความขาเข้า =====
void handleIncomingAPRS(const String &line)
{
    if (config.msg_enable == false)
        return;
    // ตัวอย่างการกรองข้อความที่มีรูปแบบ Message
    int msgPos = line.indexOf("::");
    if (msgPos <= 0)
        return;
    if (msgPos > 0)
    {
        String fromCall = line.substring(0, line.indexOf('>'));
        String payload = line.substring(msgPos + 2);

        if (payload.length() < 10)
            return;
        String toCall = payload.substring(0, 9);
        toCall.trim();

        if (fromCall.equalsIgnoreCase(toCall))
        {
            return; // ข้ามข้อความที่ส่งมาให้ตัวเอง
        }

        int colonPos = payload.indexOf(':', 9);
        if (colonPos > 0)
        {
            String message = payload.substring(colonPos + 1);
            message.trim();

            // ตรวจว่ามีหมายเลขข้อความไหม (หลังเครื่องหมาย { )
            String msgNo = "";
            int bracePos = message.indexOf('{');
            if (bracePos >= 0 && bracePos < (int)message.length() - 1)
            {
                msgNo = message.substring(bracePos + 1);
                message = message.substring(0, bracePos); // ตัดหมายเลขออกจากข้อความ
                message.trim();
            }

            if (message.startsWith("ack"))
            {
                msgNo = message.substring(3);
            }

            log_d("📩 Message from %s to %s : %s", fromCall.c_str(), toCall.c_str(), message.c_str());

            // ถ้าเป็นข้อความถึงเราเอง ให้ตอบกลับ ack
            if (toCall.equalsIgnoreCase(config.msg_mycall) && msgNo.length() > 0)
            {
                if (message.startsWith("ack"))
                {
                    msgNo = message.substring(3);
                    int i = pkgMsg_Find(fromCall.c_str(), msgNo.toInt(), false);
                    log_d("Message ACk from %s msgNo %d msgQueue %i", fromCall.c_str(), msgNo.toInt(), i);
                    if (i > -1)
                    {
                        msgQueue[i].ack = -2; // ตอบรับแล้ว
                    }
                }
                else
                {
                    String decrypted = "";
                    if (config.msg_encrypt)
                    {
                        uint8_t aes_key[16];
                        hexStringToBytes(config.msg_key, aes_key, sizeof(aes_key));
                        decrypted = aesDecryptBase64WithIV(message, aes_key, fromCall.c_str(), msgNo.toInt());
                    }
                    else
                    {
                        decrypted = message;
                    }
                    decrypted.trim();
                    if (decrypted == "")
                        return;
                    pkgMsgUpdate(fromCall.c_str(), decrypted.c_str(), msgNo.toInt(), -1, true); // RX Message
                    sendAPRSAck(fromCall, msgNo);
                    if (config.at_cmd_msg)
                    {
                        String response = handleATCommand(decrypted);
                        if (response != "")
                            sendAPRSMessage(fromCall, response, config.msg_encrypt);
                    }
                }
                event_chatMessage(false);
            }
        }
    }
}