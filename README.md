# JoltPhysicsUnity

Version 0.2.0 is a breaking 0.x release. It retains the Entity-keyed typed complete-state contract introduced in 0.1.0 and adds the 64-byte joint-definition ABI plus pure pre-mutation joint canonicalization. Fixed definitions persist local-COM anchors and a canonical inverse orientation; Distance definitions persist local-COM anchors, resolved limits, and complete spring settings. The full-state fingerprint is `0x4A4F4C5452500004`; binaries and payloads from earlier layouts are rejected.

The package is inspired by [amerkoleci](https://github.com/amerkoleci)

This package provides __work-in-progress__ bindings for [Jolt Physics v5.3.0](https://github.com/jrouwe/JoltPhysics)
specifically for Unity. It uses the Unity.Mathematics package for all numerics and Unity.Collections
package for unmanaged collections.

The active runtime surface is the minimal Entity-keyed `JPH_*` glue in `Jolt/Bindings`, plus `Jolt.LowLevel.World`.
The generated ABI exposes no BodyInterface, NarrowPhaseQuery, native body/constraint/character pointer,
or opaque recorder state. Unsupported full-joltc APIs fail at compile time instead of silently executing
no-op compatibility shells. Native body diagnostics are exposed as a bounded
`JPH_PhysicsSystem_DrawDebugLines` wireframe buffer instead of reverse-P/Invoke renderer callbacks.
Complete `PhysicsWorldState` synchronization uses typed continuation counts/capture exports and
`JPH_PhysicsSystem_ValidateFullState` / `JPH_PhysicsSystem_SyncFullStateIn`. Authored joint arrays are copied and passed through `JPH_PhysicsSystem_CanonicalizeJoints` before live mutation; the caller's arrays are never rewritten. Native restore constructs a
canonical shadow runtime in Global → Bodies → broadphase → Contacts → Constraints → Characters order,
then swaps the stable runtime slot only after every phase succeeds. Every cross-object reference crosses
the ABI as a logical Entity key while Jolt native handles remain native-only. A fingerprint export is
checked before any continuation call so managed code cannot silently use a binary with another layout.
Storage and identity adapters live above this ABI; the implementation does not call
`PhysicsSystem::SaveState` / `RestoreState`.
`NativeRingBuffer` re-strides every slot whenever slot capacity grows, including when
the existing power-of-two allocation has unused total-byte slack.
The checked-in native runtime currently targets macOS universal
(`x86_64` + `arm64`); the version-controlled `libjoltc.dylib` is the GitHub Actions
Distribution artifact. The shared deploy script can temporarily install a Debug build for
local investigation, but Distribution must be restored before committing. Other platforms must build the
same minimal glue from the canonical `Jolt.NET/native/jolt_glue.*` source before enabling the package there.

### Installation

The package is not currently published on any registries. You can [add it as a git URL](https://docs.unity3d.com/Manual/upm-ui-giturl.html)
or [embed it directly in your project](https://docs.unity3d.com/Manual/upm-embed.html).