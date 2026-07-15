namespace Jolt
{
    /// <summary>
    /// Marker implemented by authoring components that can be baked into a ColliderBlob.
    /// Native Jolt shapes are parsed and cached only by the native SyncWorldIn boundary.
    /// </summary>
    public interface IPhysicsShape
    {
    }
}
