// gui/app/lz4_frame_decoder.cpp — decode-only LZ4 (frame + block) decoder.
//
// Frame format (lz4 frame spec v1.6.x): magic 0x184D2204, FLG/BD descriptor
// bytes, optional content size, the always-present 1-byte header checksum,
// then blocks [u32 size | high bit = uncompressed | 0 = EndMark] until
// EndMark, optionally followed by the 4-byte content checksum. Block
// payloads use the LZ4 block sequence format (token → literals → u16 LE
// offset → match copy). Block checksums (FLG bit 4) and content checksums
// (FLG bit 2) are skipped, not verified. Matches may reference previously
// decoded output across blocks (blockLinked frames, DV's default); with
// independent blocks such a reference can only come from a corrupt stream,
// which a decode-only reader does not need to distinguish.

#include "lz4_frame_decoder.h"

namespace gui {
namespace {

constexpr std::uint32_t kFrameMagic = 0x184D2204;
// A decoded event packet is at most a few MB; anything claiming or producing
// more than this is a corrupt/hostile frame (unbounded reserve() would throw
// far below this, but the cap also bounds the decompression loop itself).
constexpr std::size_t kMaxDecodedBytes = 512u << 20;

// Decodes one compressed LZ4 block sequence into @p out (appended).
bool decompress_block(const std::uint8_t* src, const std::uint8_t* src_end,
                      std::vector<std::uint8_t>& out) {
    const std::uint8_t* p = src;
    for (;;) {
        if (p >= src_end) return false;
        const std::uint8_t token = *p++;
        std::size_t lit_len = token >> 4;
        if (lit_len == 15) {
            std::uint8_t b = 0;
            do {
                if (p >= src_end) return false;
                b = *p++;
                lit_len += b;
            } while (b == 255);
        }
        if (static_cast<std::size_t>(src_end - p) < lit_len) return false;
        out.insert(out.end(), p, p + lit_len);
        p += lit_len;
        if (p == src_end) {
            // Last sequence carries no match — end of block.
            return true;
        }
        if (src_end - p < 2) return false;
        const std::size_t offset = static_cast<std::size_t>(p[0]) |
                                   (static_cast<std::size_t>(p[1]) << 8);
        p += 2;
        if (offset == 0) return false;
        std::size_t match_len = (token & 0x0F) + 4;
        if ((token & 0x0F) == 15) {
            std::uint8_t b = 0;
            do {
                if (p >= src_end) return false;
                b = *p++;
                match_len += b;
            } while (b == 255);
        }
        // Matches may reference any previously decoded output of the frame
        // (blockLinked semantics: up to the window size across blocks).
        if (offset > out.size()) return false;
        // Forward byte-by-byte copy: references into the freshly written
        // tail implement the standard overlapping-run semantics.
        const std::size_t match_pos = out.size();
        out.resize(match_pos + match_len);
        for (std::size_t i = 0; i < match_len; ++i) {
            out[match_pos + i] = out[match_pos + i - offset];
        }
        if (p == src_end) return true; // block ended on a match
    }
}

} // namespace

bool lz4_decompress_frame(const std::uint8_t* data, std::size_t size,
                          std::vector<std::uint8_t>& out, std::string& error) {
    const std::uint8_t* p = data;
    const std::uint8_t* end = data + size;
    if (size < 7) {
        error = "LZ4 frame too short";
        return false;
    }
    while (p + 7 <= end) {
        const std::uint32_t magic = static_cast<std::uint32_t>(p[0]) |
                                    (static_cast<std::uint32_t>(p[1]) << 8) |
                                    (static_cast<std::uint32_t>(p[2]) << 16) |
                                    (static_cast<std::uint32_t>(p[3]) << 24);
        if (magic != kFrameMagic) {
            error = "bad LZ4 frame magic";
            return false;
        }
        p += 4;
        const std::uint8_t flg = *p++;
        const std::uint8_t bd = *p++;
        if (((flg >> 6) & 0x03) != 0x01) {
            error = "unsupported LZ4 frame version";
            return false;
        }
        if (flg & 0x02) { // reserved bit set
            error = "invalid LZ4 FLG byte";
            return false;
        }
        const bool has_content_size = flg & 0x08;
        const bool has_block_checksum = flg & 0x10;
        const bool has_content_checksum = flg & 0x04;
        const bool has_dict_id = flg & 0x01;
        const unsigned bmax_code = (bd >> 4) & 0x07;
        if (bmax_code == 0 || (bd & 0x8F) != 0) {
            error = "invalid LZ4 BD byte";
            return false;
        }
        const std::uint32_t block_max = 1u << (8 + 2 * bmax_code);
        std::uint64_t content_size = 0;
        if (has_content_size) {
            if (end - p < 8) {
                error = "truncated LZ4 frame header";
                return false;
            }
            for (int i = 0; i < 8; ++i) {
                content_size |= static_cast<std::uint64_t>(*p++) << (8 * i);
            }
            if (content_size > kMaxDecodedBytes) {
                error = "unreasonable LZ4 content size";
                return false;
            }
        }
        if (has_dict_id) {
            if (end - p < 4) {
                error = "truncated LZ4 frame header";
                return false;
            }
            p += 4; // dictionary id unused
        }
        // The 1-byte header checksum is unconditionally present after the
        // descriptor (xxh32 of the header, second byte; not verified here).
        if (p >= end) {
            error = "truncated LZ4 frame header";
            return false;
        }
        ++p;
        if (content_size > 0) {
            out.reserve(out.size() + static_cast<std::size_t>(content_size));
        }
        // Blocks.
        for (;;) {
            if (end - p < 4) {
                error = "truncated LZ4 block header";
                return false;
            }
            std::uint32_t bsize = static_cast<std::uint32_t>(p[0]) |
                                  (static_cast<std::uint32_t>(p[1]) << 8) |
                                  (static_cast<std::uint32_t>(p[2]) << 16) |
                                  (static_cast<std::uint32_t>(p[3]) << 24);
            p += 4;
            if (bsize == 0) break; // EndMark
            const bool uncompressed = bsize & 0x80000000u;
            bsize &= 0x7FFFFFFFu;
            if (bsize > block_max || static_cast<std::size_t>(end - p) < bsize) {
                error = "invalid LZ4 block size";
                return false;
            }
            const std::size_t out_before = out.size();
            if (uncompressed) {
                out.insert(out.end(), p, p + bsize);
            } else if (!decompress_block(p, p + bsize, out)) {
                error = "corrupt LZ4 block";
                return false;
            }
            if (out.size() - out_before > block_max ||
                out.size() > kMaxDecodedBytes) {
                error = "LZ4 output exceeds size limit";
                return false;
            }
            p += bsize;
            if (has_block_checksum) {
                if (end - p < 4) {
                    error = "truncated LZ4 block checksum";
                    return false;
                }
                p += 4; // xxh32 block checksum not verified (decode-only)
            }
        }
        if (has_content_checksum) {
            if (end - p < 4) {
                error = "truncated LZ4 content checksum";
                return false;
            }
            p += 4; // xxh32 content checksum not verified (decode-only)
        }
    }
    if (p != end) {
        error = "trailing bytes after LZ4 frame";
        return false;
    }
    return true;
}

} // namespace gui
