# Using LCM with Unity

- Using the `1.4.0` version from [here](https://github.com/lcm-proj/lcm)
- Although LCM has C# bindings, there aren't any examples of using it with Unity from what I can tell.
- Copied `lcm/lcm-dotnet` into this folder and created an assembly definition for it.
- Had to delete the `lcm-server` folder. I don't think we need it anyways, just the message-related library.
- After generating C# lcmtypes with `lcm-gen --csharp *.lcm`, copy them into the `lcmtypes` directory here.
