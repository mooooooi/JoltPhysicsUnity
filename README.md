# JoltPhysicsUnity

The package is inspired by [amerkoleci](https://github.com/amerkoleci)

This package provides __work-in-progress__ bindings for [Jolt Physics v5.3.0](https://github.com/jrouwe/JoltPhysics)
specifically for Unity. It uses the Unity.Mathematics package for all numerics and Unity.Collections
package for unmanaged collections.

The active runtime surface is the minimal `JPH_*` glue in `Jolt/Bindings`, plus `Jolt.LowLevel.World`.
It exposes raw unmanaged handles for Unity Jobs/Burst and does not ship the former managed
`LegacyCompat` façade. Unsupported full-joltc APIs fail at compile time instead of silently executing
no-op compatibility shells. Native body diagnostics are exposed as a bounded
`JPH_PhysicsSystem_DrawDebugLines` wireframe buffer instead of reverse-P/Invoke renderer callbacks.
The checked-in native runtime currently targets macOS universal
(`x86_64` + `arm64`); the version-controlled `libjoltc.dylib` is the GitHub Actions
Distribution/Release artifact. The shared deploy script can temporarily install a Debug build for
local investigation, but Release must be restored before committing. Other platforms must build the
same minimal glue from the canonical `Jolt.NET/native/jolt_glue.*` source before enabling the package there.

### Installation

The package is not currently published on any registries. You can [add it as a git URL](https://docs.unity3d.com/Manual/upm-ui-giturl.html)
or [embed it directly in your project](https://docs.unity3d.com/Manual/upm-embed.html).