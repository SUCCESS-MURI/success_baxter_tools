# success_baxter_tools
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

A collections of python modules/functions useful when working with baxter

## List of Modules/Functions
### Motion
* IK solver
```
from success_baxter_tools.motion import solve_IK

joint_angles = solve_IK(arm, pose)
```
* save/replay arm posture
```
from success_baxter_tools.motion import save_posture, move_to_posture

save_posture(posture_name, button_control=True, arm=None, record_path="posture_records.yaml")
move_to_posture(posture_name, record_path="posture_records.yaml")

```

## Past Contributors
- Joe Connolly - 07/2018