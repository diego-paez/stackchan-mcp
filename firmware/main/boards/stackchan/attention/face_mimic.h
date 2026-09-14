// face_mimic.h — the robot's face, answering the person's.
//
// The head follows a face; this decides what the screen does about it. Same
// rules as everything else in this directory: no ESP-IDF, no allocation, no
// direct call into the display, so the whole policy can be stepped and
// asserted on a laptop.
//
// THE TWO VOCABULARIES DO NOT MATCH, AND CANNOT
// The pipeline reports seven affects — anger, disgust, fear, happy, neutral,
// sad, surprise (chatbot/models/emotion/base.py). The avatar has six faces —
// idle, happy, thinking, sad, surprised, embarrassed (stackchan.cc:6150).
// There is no identity map between them, so there is a table, and the table
// is the interesting part of this file.
//
// MIRRORING IS NOT THE GOAL
// A robot that mirrors affect indiscriminately is not empathetic, it is a
// loop. Anger reflected back at an angry child escalates; fear reflected back
// confirms that there is something to be afraid of. The default policy
// mirrors what is safe to mirror and answers the rest:
//
//   happy    -> happy         mirror. This is the one everybody expects.
//   surprise -> surprised     mirror. Shared surprise is joint attention.
//   sad      -> sad           mirror. A sad child met with a cheerful face
//                             reads as not having been heard.
//   anger    -> thinking      answer. Attentive and taking it seriously,
//                             rather than matching the escalation.
//   fear     -> sad           answer. Concern, not alarm.
//   disgust  -> embarrassed   answer. The nearest thing the avatar has to
//                             "that was awkward".
//   neutral  -> idle          rest.
//
// Those seven rows are a pedagogical judgement, not an engineering one. The
// host tests cannot tell you whether they are right for a child — they can
// only prove the table is applied consistently, that the face cannot flicker,
// and that it cannot get stuck. Changing a row is a one-line change here, on
// purpose.
#pragma once

#include <cstdint>

namespace stackchan {
namespace attention {

// The only faces the avatar actually has. The strings are the ones
// SetAvatarExpression() already parses; a seventh value would render nothing.
enum class AvatarFace : uint8_t {
    kIdle = 0,
    kHappy,
    kThinking,
    kSad,
    kSurprised,
    kEmbarrassed,
};

const char* ToString(AvatarFace f);

// Name -> face, mirroring StackChanBoard::FaceNameToIndex() exactly. Returns
// false for a name the avatar cannot render, which is not a theoretical case:
// the greeting script shipped "neutral" for a while, a perfectly sensible
// word that FaceNameToIndex() has never known, and the effect on hardware is
// silent — the expression is logged as deferred and the previous face stays.
// Anything that names a face should be checked against this.
bool FaceFromName(const char* name, AvatarFace* out);

// The affect vocabulary the pipeline speaks.
enum class Affect : uint8_t {
    kUnknown = 0,
    kAnger,
    kDisgust,
    kFear,
    kHappy,
    kNeutral,
    kSad,
    kSurprise,
};

const char* ToString(Affect a);

// Label string -> Affect. Case-insensitive, and tolerant of the aliases the
// speech models actually emit: the pipeline already has to map "angry" onto
// "anger" and "joy" onto "happy" (models/emotion/speech_hf.py), and a label
// that has crossed a network is exactly where a spelling difference shows up.
// Anything unrecognised is kUnknown, which is treated as no evidence rather
// than as neutral — "I did not understand that" and "they are calm" are very
// different things to put on a robot's face.
Affect AffectFromLabel(const char* label);

// The response table. One field per observed affect; see the header comment
// for why they are not all mirrors.
struct MimicPolicy {
    AvatarFace on_anger = AvatarFace::kThinking;
    AvatarFace on_disgust = AvatarFace::kEmbarrassed;
    AvatarFace on_fear = AvatarFace::kSad;
    AvatarFace on_happy = AvatarFace::kHappy;
    AvatarFace on_neutral = AvatarFace::kIdle;
    AvatarFace on_sad = AvatarFace::kSad;
    AvatarFace on_surprise = AvatarFace::kSurprised;
};

// Pure lookup. kUnknown maps to kIdle, which is the resting face, not a
// response to anything.
AvatarFace ResponseTo(Affect a, const MimicPolicy& policy);

struct MimicConfig {
    // Below this, the observation is not evidence of anything. The pipeline
    // returns a uniform distribution when a model fails, and 1/7 = 0.143 must
    // not move the robot's face.
    float min_confidence = 0.45f;

    // A reading is confirmed, and the face may change, by ANY of three
    // routes. Three, rather than one, because this component cannot know how
    // fast its source is: a per-frame face-emotion stream arrives at 5 Hz and
    // a transcribe endpoint arrives once per utterance, seconds apart, and a
    // rule tuned for either is wrong for the other.
    //
    //   by count      — this many agreeing readings in a row. At 5 Hz that is
    //                   400 ms; at one per utterance it is a whole
    //                   conversational turn of lag, which is why it is not
    //                   the only route.
    uint32_t confirmations = 2;

    //   by confidence — a reading this strong stands on its own. Set from
    //                   what the models actually return: a clear utterance
    //                   comes back around 0.65-0.85 from a softmax over
    //                   seven classes, so a bar at 0.85 means almost nothing
    //                   a child says alone ever moves the face. sim_mimic
    //                   showed the cost — three seconds wearing "surprised"
    //                   while she explained her tower had fallen over.
    float instant_confidence = 0.65f;

    //   by dwell      — a reading nothing has contradicted for this long.
    //                   This is what lets a single utterance land without
    //                   lowering the confidence bar to where noise passes,
    //                   and a contradiction resets it, so alternating
    //                   readings confirm by no route at all.
    uint32_t confirm_dwell_ms = 2500;

    // A floor on how fast the face can change, whatever the evidence says.
    uint32_t min_hold_ms = 1200;

    // No accepted observation for this long and the face returns to idle. A
    // robot still wearing the last thing it was told, four minutes later, is
    // not expressive — it is stuck.
    uint32_t stale_ms = 6000;

    bool enabled = true;
};

// Counters, because the failure that matters is "it silently never changes"
// and that looks identical to a calm room unless something is counting.
struct MimicStats {
    uint32_t observations = 0;   // every call to observe()
    uint32_t unknown = 0;        // label not in the vocabulary
    uint32_t weak = 0;           // below min_confidence, or NaN
    uint32_t accepted = 0;       // fed into the confirmation logic
    uint32_t changes = 0;        // times the face actually changed
    uint32_t held = 0;           // changes suppressed by min_hold_ms
    uint32_t decays = 0;         // returns to idle through staleness
};

// Observations in, one face out. Holds no sink: it computes what the face
// should be and reports when that answer changes, and the owner does the
// talking to the display. That is what keeps this testable and what keeps
// exactly one thing writing to the screen.
class FaceMimic {
public:
    FaceMimic() = default;
    FaceMimic(const MimicConfig& cfg, const MimicPolicy& policy)
        : cfg_(cfg), policy_(policy) {}

    // Feed an observation of the person. Safe to call at any rate, including
    // not at all.
    void observe(const char* label, float confidence, uint32_t now_ms);
    void observe(Affect a, float confidence, uint32_t now_ms);

    // Step the policy. Returns true when the face changed on this tick and
    // the owner should tell the display; false is the overwhelmingly common
    // answer and costs nothing.
    bool update(uint32_t now_ms);

    AvatarFace face() const { return face_; }
    const char* faceName() const { return ToString(face_); }
    Affect affect() const { return affect_; }
    MimicStats stats() const { return stats_; }

    // Two different things can stop the face, and conflating them was a bug.
    //
    // SUPPRESSED: something else owns the screen — the greeting while it
    // plays, the mouth while the robot speaks. The policy keeps running and
    // the answer stays current; nothing is reported. When suppression lifts,
    // a face that drifted in the meantime is delivered once, so the screen is
    // never left showing a stale answer.
    void setSuppressed(bool on) { suppressed_ = on; }
    bool suppressed() const { return suppressed_; }

    // HALTED: the robot has been stopped. Observations are dropped on the
    // floor rather than accumulated, the face goes to idle, and that idle IS
    // reported — the same rule the greeting follows, that a stopped robot
    // does not keep performing. Without the distinction, provoking a stopped
    // robot leaves it wearing an expression the moment it is released.
    void setHalted(bool on, uint32_t now_ms);
    bool halted() const { return halted_; }

    // Forget everything and rest. setHalted() uses it; an owner can too.
    void reset(uint32_t now_ms);

    void setEnabled(bool on) { cfg_.enabled = on; }
    bool enabled() const { return cfg_.enabled; }

    void setConfig(const MimicConfig& cfg) { cfg_ = cfg; }
    MimicConfig config() const { return cfg_; }
    void setPolicy(const MimicPolicy& p) { policy_ = p; }

private:
    MimicConfig cfg_;
    MimicPolicy policy_;

    AvatarFace face_ = AvatarFace::kIdle;       // what the face should be
    AvatarFace emitted_ = AvatarFace::kIdle;    // what the display was last told
    Affect affect_ = Affect::kUnknown;          // the confirmed reading

    Affect pending_ = Affect::kUnknown;
    uint32_t pending_count_ = 0;
    uint32_t pending_since_ms_ = 0;
    float pending_conf_ = 0.0f;

    bool have_obs_ = false;
    uint32_t last_obs_ms_ = 0;
    uint32_t last_change_ms_ = 0;
    // min_hold_ms is a floor between changes, not a delay before the first
    // one: a robot that cannot react for 1.2 s after boot is just slow.
    bool ever_changed_ = false;
    bool suppressed_ = false;
    bool halted_ = false;
    MimicStats stats_;
};

}  // namespace attention
}  // namespace stackchan
