#include "face_mimic.h"

#include <cmath>
#include <cstring>

namespace stackchan {
namespace attention {
namespace {

// Case-insensitive compare over ASCII. <cctype>'s tolower takes an int whose
// value must be representable as unsigned char, and a label off the wire can
// carry bytes that are not; doing the arithmetic here avoids that trap.
bool EqualsIgnoreCase(const char* a, const char* b) {
    while (*a != '\0' && *b != '\0') {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb - 'A' + 'a');
        if (ca != cb) return false;
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

struct LabelAlias {
    const char* text;
    Affect affect;
};

// The canonical seven, plus every spelling these models have been seen to
// emit. Matching by name rather than by index is what lets a model be swapped
// without silently renumbering the robot's feelings.
constexpr LabelAlias kAliases[] = {
    {"anger", Affect::kAnger},       {"angry", Affect::kAnger},
    {"ang", Affect::kAnger},         {"mad", Affect::kAnger},
    {"disgust", Affect::kDisgust},   {"disgusted", Affect::kDisgust},
    {"dis", Affect::kDisgust},
    {"fear", Affect::kFear},         {"fearful", Affect::kFear},
    {"scared", Affect::kFear},       {"afraid", Affect::kFear},
    {"happy", Affect::kHappy},       {"happiness", Affect::kHappy},
    {"joy", Affect::kHappy},         {"hap", Affect::kHappy},
    {"neutral", Affect::kNeutral},   {"calm", Affect::kNeutral},
    {"neu", Affect::kNeutral},
    {"sad", Affect::kSad},           {"sadness", Affect::kSad},
    {"surprise", Affect::kSurprise}, {"surprised", Affect::kSurprise},
    {"sur", Affect::kSurprise},
};

}  // namespace

const char* ToString(AvatarFace f) {
    switch (f) {
        case AvatarFace::kIdle: return "idle";
        case AvatarFace::kHappy: return "happy";
        case AvatarFace::kThinking: return "thinking";
        case AvatarFace::kSad: return "sad";
        case AvatarFace::kSurprised: return "surprised";
        case AvatarFace::kEmbarrassed: return "embarrassed";
    }
    return "idle";
}

bool FaceFromName(const char* name, AvatarFace* out) {
    if (name == nullptr) return false;
    static constexpr AvatarFace kAll[] = {
        AvatarFace::kIdle,      AvatarFace::kHappy,      AvatarFace::kThinking,
        AvatarFace::kSad,       AvatarFace::kSurprised,  AvatarFace::kEmbarrassed,
    };
    for (AvatarFace f : kAll) {
        if (std::strcmp(name, ToString(f)) == 0) {
            if (out != nullptr) *out = f;
            return true;
        }
    }
    return false;
}

const char* ToString(Affect a) {
    switch (a) {
        case Affect::kUnknown: return "unknown";
        case Affect::kAnger: return "anger";
        case Affect::kDisgust: return "disgust";
        case Affect::kFear: return "fear";
        case Affect::kHappy: return "happy";
        case Affect::kNeutral: return "neutral";
        case Affect::kSad: return "sad";
        case Affect::kSurprise: return "surprise";
    }
    return "unknown";
}

Affect AffectFromLabel(const char* label) {
    if (label == nullptr || label[0] == '\0') return Affect::kUnknown;
    for (const LabelAlias& a : kAliases) {
        if (EqualsIgnoreCase(label, a.text)) return a.affect;
    }
    return Affect::kUnknown;
}

AvatarFace ResponseTo(Affect a, const MimicPolicy& policy) {
    switch (a) {
        case Affect::kAnger: return policy.on_anger;
        case Affect::kDisgust: return policy.on_disgust;
        case Affect::kFear: return policy.on_fear;
        case Affect::kHappy: return policy.on_happy;
        case Affect::kNeutral: return policy.on_neutral;
        case Affect::kSad: return policy.on_sad;
        case Affect::kSurprise: return policy.on_surprise;
        case Affect::kUnknown: break;
    }
    return AvatarFace::kIdle;
}

void FaceMimic::observe(const char* label, float confidence, uint32_t now_ms) {
    observe(AffectFromLabel(label), confidence, now_ms);
}

void FaceMimic::observe(Affect a, float confidence, uint32_t now_ms) {
    ++stats_.observations;

    // A stopped robot is not gathering evidence about anybody.
    if (halted_) return;

    if (a == Affect::kUnknown) {
        ++stats_.unknown;
        return;
    }
    // Written as !(>=) so NaN lands here rather than passing: a confidence
    // that is not a number is not a confident detection.
    if (!(confidence >= cfg_.min_confidence)) {
        ++stats_.weak;
        return;
    }

    ++stats_.accepted;
    have_obs_ = true;
    last_obs_ms_ = now_ms;
    pending_conf_ = confidence;

    if (a == pending_) {
        ++pending_count_;
    } else {
        pending_ = a;
        pending_count_ = 1;
        pending_since_ms_ = now_ms;
    }
}

bool FaceMimic::update(uint32_t now_ms) {
    if (halted_) {
        face_ = AvatarFace::kIdle;
    } else if (!cfg_.enabled) {
        // Disabled means resting, not frozen: whatever is on the screen
        // stops being this component's opinion.
        face_ = AvatarFace::kIdle;
        affect_ = Affect::kUnknown;
        have_obs_ = false;
        pending_ = Affect::kUnknown;
        pending_count_ = 0;
    } else {
        // 1. Staleness. Unsigned subtraction is deliberate and correct across
        //    the 49-day millisecond wrap, the same way the rest of this
        //    directory handles time.
        if (have_obs_ && (now_ms - last_obs_ms_) >= cfg_.stale_ms) {
            have_obs_ = false;
            pending_ = Affect::kUnknown;
            pending_count_ = 0;
            if (affect_ != Affect::kUnknown) ++stats_.decays;
            affect_ = Affect::kUnknown;
        }

        // 2. Promote a pending reading once it has been confirmed — either by
        //    repetition or by being confident enough to stand on its own.
        if (pending_ != Affect::kUnknown && pending_ != affect_) {
            const bool by_count = pending_count_ >= cfg_.confirmations;
            const bool by_confidence = pending_conf_ >= cfg_.instant_confidence;
            const bool by_dwell =
                cfg_.confirm_dwell_ms > 0 &&
                (now_ms - pending_since_ms_) >= cfg_.confirm_dwell_ms;
            if (by_count || by_confidence || by_dwell) affect_ = pending_;
        }

        // 3. The face the confirmed reading asks for. Two affects may map to
        //    the same face (fear and sad both answer with sad), in which case
        //    nothing below this line happens at all.
        const AvatarFace want = ResponseTo(affect_, policy_);
        if (want != face_) {
            if (ever_changed_ && (now_ms - last_change_ms_) < cfg_.min_hold_ms) {
                ++stats_.held;
            } else {
                face_ = want;
                last_change_ms_ = now_ms;
                ever_changed_ = true;
                ++stats_.changes;
            }
        }
    }

    if (suppressed_) return false;
    if (face_ == emitted_) return false;
    emitted_ = face_;
    return true;
}

void FaceMimic::setHalted(bool on, uint32_t now_ms) {
    if (on == halted_) return;
    halted_ = on;
    reset(now_ms);
}

void FaceMimic::reset(uint32_t now_ms) {
    face_ = AvatarFace::kIdle;
    affect_ = Affect::kUnknown;
    pending_ = Affect::kUnknown;
    pending_count_ = 0;
    pending_since_ms_ = now_ms;
    pending_conf_ = 0.0f;
    have_obs_ = false;
    last_obs_ms_ = now_ms;
    last_change_ms_ = now_ms;
    ever_changed_ = false;
}

}  // namespace attention
}  // namespace stackchan
