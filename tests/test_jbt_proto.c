#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "jbt_proto.h"

/* Runs every wire byte through a decoder and returns how many frames came
 * out; the last one is copied to `last`. */
static size_t feed_all(jbt_decoder_t *decoder, const uint8_t *wire, size_t length,
                       jbt_frame_t *last, uint8_t *payload_copy)
{
    size_t frames = 0U;
    for (size_t i = 0; i < length; ++i) {
        jbt_frame_t frame;
        if (jbt_decoder_feed(decoder, wire[i], &frame)) {
            ++frames;
            if (last != NULL) {
                *last = frame;
                if (payload_copy != NULL && frame.len > 0U) {
                    memcpy(payload_copy, frame.payload, frame.len);
                    last->payload = payload_copy;
                }
            }
        }
    }
    return frames;
}

static void test_the_crc_is_ccitt_false(void)
{
    /* The check value every CRC catalogue lists for this variant - if the
     * host and the module ever disagree on the polynomial, this is the test
     * that says which side is right. */
    assert(jbt_crc16((const uint8_t *)"123456789", 9U) == 0x29B1U);
    assert(jbt_crc16((const uint8_t *)"", 0U) == 0xFFFFU);
}

static void test_a_frame_survives_the_round_trip(void)
{
    const uint8_t payload[] = {1, 2, 3, 0xC0, 0xDB, 0xC0, 0xDC, 0xDD, 0};
    const jbt_frame_t frame = {
        .type = JBT_MSG_COVER_DATA, .flags = JBT_FLAG_WANT_ACK, .seq = 0x5A,
        .len = sizeof(payload), .payload = payload,
    };
    uint8_t wire[JBT_WIRE_MAX];
    const size_t n = jbt_frame_encode(&frame, wire, sizeof(wire));
    assert(n > 0U);
    assert(wire[0] == JBT_SLIP_END && wire[n - 1U] == JBT_SLIP_END);
    /* Nothing between the ENDs may be an END: that is what the escaping is for. */
    for (size_t i = 1; i + 1U < n; ++i) assert(wire[i] != JBT_SLIP_END);
    /* Three reserved bytes in the payload cost three extra wire bytes. */
    assert(n == 2U + JBT_HEADER_SIZE + sizeof(payload) + JBT_CRC_SIZE + 3U);

    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    jbt_frame_t got;
    uint8_t copy[JBT_PAYLOAD_MAX];
    assert(feed_all(&decoder, wire, n, &got, copy) == 1U);
    assert(got.type == JBT_MSG_COVER_DATA);
    assert(got.flags == JBT_FLAG_WANT_ACK);
    assert(got.seq == 0x5A);
    assert(got.len == sizeof(payload));
    assert(memcmp(got.payload, payload, sizeof(payload)) == 0);
    assert(decoder.dropped == 0U);
}

static void test_an_empty_payload_is_a_frame(void)
{
    const jbt_frame_t frame = {.type = JBT_MSG_PING, .seq = 1};
    uint8_t wire[JBT_WIRE_MAX];
    const size_t n = jbt_frame_encode(&frame, wire, sizeof(wire));
    assert(n == 2U + JBT_HEADER_SIZE + JBT_CRC_SIZE);
    /* The same bytes tools/jbt.py pins in its selftest: the one vector both
     * implementations agree on byte for byte. */
    const uint8_t expected[] = {0xC0, 0x0D, 0x00, 0x01, 0x00, 0x00, 0x46, 0x07, 0xC0};
    assert(memcmp(wire, expected, sizeof(expected)) == 0);
    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    jbt_frame_t got;
    assert(feed_all(&decoder, wire, n, &got, NULL) == 1U);
    assert(got.type == JBT_MSG_PING && got.len == 0U);
}

static void test_the_decoder_resynchronises_after_garbage(void)
{
    /* A reset of either side mid-frame, then a good frame: the garbage is
     * counted once and the good frame comes through. Then noise that happens
     * to contain an END in the middle of nothing - back-to-back ENDs are not
     * frames and not errors. */
    const jbt_frame_t frame = {.type = JBT_MSG_GET_STATUS, .seq = 7};
    uint8_t wire[JBT_WIRE_MAX];
    const size_t n = jbt_frame_encode(&frame, wire, sizeof(wire));

    const uint8_t garbage[] = {0x00, 0xFF, 0x12, 0xC0, 0xC0, 0xC0, 0xDB, 0x00, 0xC0};
    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    assert(feed_all(&decoder, garbage, sizeof(garbage), NULL, NULL) == 0U);
    /* "00 FF 12" is too short; "DB 00" is a bad escape. */
    assert(decoder.dropped == 2U);
    jbt_frame_t got;
    assert(feed_all(&decoder, wire, n, &got, NULL) == 1U);
    assert(got.type == JBT_MSG_GET_STATUS && got.seq == 7);
    assert(decoder.dropped == 2U);
}

static void test_a_corrupted_byte_is_dropped_not_delivered(void)
{
    const uint8_t payload[] = {10, 20, 30};
    const jbt_frame_t frame = {.type = JBT_MSG_SET_VOLUME, .len = 3, .payload = payload};
    uint8_t wire[JBT_WIRE_MAX];
    const size_t n = jbt_frame_encode(&frame, wire, sizeof(wire));
    wire[JBT_HEADER_SIZE + 2U] ^= 0x01U; /* a payload byte, not the framing */
    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    assert(feed_all(&decoder, wire, n, NULL, NULL) == 0U);
    assert(decoder.dropped == 1U);

    /* And a frame whose length field lies about its payload. */
    const size_t m = jbt_frame_encode(&frame, wire, sizeof(wire));
    wire[1U + 3U] = 5; /* len low byte: claims 5, carries 3 */
    assert(feed_all(&decoder, wire, m, NULL, NULL) == 0U);
    assert(decoder.dropped == 2U);
}

static void test_an_oversized_frame_is_refused_on_both_ends(void)
{
    uint8_t payload[JBT_PAYLOAD_MAX + 1U];
    memset(payload, 0x11, sizeof(payload));
    jbt_frame_t frame = {.type = JBT_MSG_COVER_DATA, .len = JBT_PAYLOAD_MAX + 1U, .payload = payload};
    uint8_t wire[JBT_WIRE_MAX];
    assert(jbt_frame_encode(&frame, wire, sizeof(wire)) == 0U);

    /* The largest legal payload does fit, even when every byte needs an
     * escape - that is what JBT_WIRE_MAX is sized for. */
    memset(payload, 0xC0, sizeof(payload));
    frame.len = JBT_PAYLOAD_MAX;
    const size_t n = jbt_frame_encode(&frame, wire, sizeof(wire));
    assert(n > 0U && n <= JBT_WIRE_MAX);
    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    jbt_frame_t got;
    assert(feed_all(&decoder, wire, n, &got, NULL) == 1U);
    assert(got.len == JBT_PAYLOAD_MAX);

    /* A stream longer than any frame is discarded up to the next END and
     * counted once, not once per byte. */
    uint8_t flood[JBT_FRAME_MAX + 50U];
    memset(flood, 0x22, sizeof(flood));
    flood[sizeof(flood) - 1U] = JBT_SLIP_END;
    assert(feed_all(&decoder, flood, sizeof(flood), NULL, NULL) == 0U);
    assert(decoder.dropped == 1U);
    /* And a small buffer for the encoder is simply refused. */
    uint8_t tiny[8];
    assert(jbt_frame_encode(&frame, tiny, sizeof(tiny)) == 0U);
}

static void test_two_frames_in_one_stream_share_one_end(void)
{
    /* Encoded back to back, the trailing END of the first frame doubles as
     * the leading END of the second; the decoder must not need two. */
    const jbt_frame_t a = {.type = JBT_MSG_PING, .seq = 1};
    const jbt_frame_t b = {.type = JBT_MSG_GET_STATUS, .seq = 2};
    uint8_t wire[2U * JBT_WIRE_MAX];
    size_t n = jbt_frame_encode(&a, wire, sizeof(wire));
    n += jbt_frame_encode(&b, &wire[n - 1U], sizeof(wire) - n + 1U) - 1U;
    jbt_decoder_t decoder;
    jbt_decoder_init(&decoder);
    jbt_frame_t got;
    assert(feed_all(&decoder, wire, n, &got, NULL) == 2U);
    assert(got.seq == 2);
}

static void test_the_status_and_tlv_records_read_back(void)
{
    uint8_t buffer[128];
    jbt_writer_t writer;
    jbt_writer_init(&writer, buffer, sizeof(buffer));
    const jbt_status_t status = {
        .mode = JBT_MODE_SINK, .connection = JBT_CONN_CONNECTED,
        .peer = {0x3D, 0xAB, 0x55, 0xFA, 0x58, 0xFC}, .codec = JBT_CODEC_SBC,
        .sample_rate = 44100U, .play = JBT_PLAY_PLAYING, .volume = 100,
    };
    assert(jbt_put_status(&writer, &status));
    assert(writer.length == JBT_STATUS_FIXED_SIZE);
    assert(jbt_put_tlv_string(&writer, JBT_TAG_PEER_NAME, "Ден's phone"));
    assert(jbt_put_tlv_u32(&writer, JBT_TAG_DURATION_MS, 214000U));
    assert(!writer.overflow);

    jbt_reader_t reader;
    jbt_reader_init(&reader, buffer, writer.length);
    jbt_status_t got;
    assert(jbt_get_status(&reader, &got));
    assert(got.mode == JBT_MODE_SINK && got.sample_rate == 44100U && got.volume == 100);
    assert(memcmp(got.peer, status.peer, 6) == 0);

    uint8_t tag;
    const uint8_t *value;
    uint8_t length;
    char name[32];
    assert(jbt_get_tlv(&reader, &tag, &value, &length));
    assert(tag == JBT_TAG_PEER_NAME);
    jbt_tlv_to_string(value, length, name, sizeof(name));
    assert(strcmp(name, "Ден's phone") == 0);
    uint32_t duration = 0;
    assert(jbt_get_tlv(&reader, &tag, &value, &length));
    assert(tag == JBT_TAG_DURATION_MS && jbt_tlv_to_u32(value, length, &duration));
    assert(duration == 214000U);
    assert(!jbt_get_tlv(&reader, &tag, &value, &length));

    /* A record that runs past the payload is refused, not read off the end. */
    const uint8_t truncated[] = {JBT_TAG_TITLE, 10, 'a', 'b'};
    jbt_reader_init(&reader, truncated, sizeof(truncated));
    assert(!jbt_get_tlv(&reader, &tag, &value, &length));
}

static void test_long_strings_clip_on_a_utf8_boundary(void)
{
    /* 130 Cyrillic letters are 260 bytes: over the 255 a record holds. The
     * clip lands on a letter boundary, never inside one. */
    char title[300];
    size_t n = 0U;
    for (int i = 0; i < 130; ++i) {
        title[n++] = (char)0xD0;
        title[n++] = (char)0xB0; /* "а" */
    }
    title[n] = '\0';
    uint8_t buffer[300];
    jbt_writer_t writer;
    jbt_writer_init(&writer, buffer, sizeof(buffer));
    assert(jbt_put_tlv_string(&writer, JBT_TAG_TITLE, title));
    assert(buffer[1] == 254); /* 127 whole letters, not 255 bytes */

    /* And the reader's own clip into a small buffer does the same. */
    char small[6];
    jbt_tlv_to_string(&buffer[2], buffer[1], small, sizeof(small));
    assert(strlen(small) == 4); /* two letters; a fifth byte would be half a letter */

    /* A writer that runs out of room says so once and stays refused. */
    uint8_t tiny[4];
    jbt_writer_init(&writer, tiny, sizeof(tiny));
    assert(jbt_put_u32(&writer, 1U));
    assert(!jbt_put_u8(&writer, 1U));
    assert(writer.overflow);
    assert(!jbt_put_u8(&writer, 1U));
}

static void test_message_names_cover_both_directions(void)
{
    assert(strcmp(jbt_msg_name(JBT_MSG_PING), "PING") == 0);
    assert(strcmp(jbt_msg_name(JBT_MSG_COVER_DATA), "COVER_DATA") == 0);
    assert(strcmp(jbt_msg_name(JBT_MSG_ACK), "ACK") == 0);
    assert(strcmp(jbt_msg_name(0x33), "?") == 0);
}

int main(void)
{
    test_the_crc_is_ccitt_false();
    test_a_frame_survives_the_round_trip();
    test_an_empty_payload_is_a_frame();
    test_the_decoder_resynchronises_after_garbage();
    test_a_corrupted_byte_is_dropped_not_delivered();
    test_an_oversized_frame_is_refused_on_both_ends();
    test_two_frames_in_one_stream_share_one_end();
    test_the_status_and_tlv_records_read_back();
    test_long_strings_clip_on_a_utf8_boundary();
    test_message_names_cover_both_directions();
    puts("jbt_proto tests passed");
    return 0;
}
