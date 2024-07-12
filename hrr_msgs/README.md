# HRR_msgs

The HR-Recycler cobot custom msgs package

> **NOTE**
>
> This package assumes the following to be given
>
> **tool-changer** We assume assumes that
>
> - the goal poses for each tool are unique
> - the goal poses for each tool are known
> - each tool is unique to the system
>
> **motion planner**:
> any action / skill assumes that **the motion planner subscribes to additional topics from the perception module** such that the collision scene can be updated online. Thus, the required input for the task-planner is reduced.
