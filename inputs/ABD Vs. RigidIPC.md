# Summary

| Input                | ABD (s) | RigidIPC (s)  | Speedup  | Comment                                                                        |
|----------------------|---------|---------------|----------|--------------------------------------------------------------------------------|
| USB                  | 4.01643 | 13.0569       |  3.25    |                                                                                |
| Peg Insertion        | 3.3278  | 16.5769       |  4.98    |                                                                                |
| Gear Offset          | 298.745 | 2337.55       |  7.82    | RigidIPC time step is 1/10's ABD                                               |
| Gear Offset Rotating | 320.0   | 5813.7        |  18.16   |                                                                                |
| BNC Crimp            | 43.8    | 109.0         |  2.49    | RigidIPC time step is 1/10's ABD. RigidIPC fails even with very small time step|
| DSUB                 | 2.91768 | 99.1393       |  33.98   | RigidIPC time step is 1/10's ABD. RigidIPC fails even with very small time step|


All experiments run on an Intel(R) Xeon(R) Gold 5218 CPU @ 2.30 GHz (2 processors) with 64.0 GB of RAM and 64-bit Windows 10, except for the `Gear Offset Rotating` which run on 13th Gen Intel(R) Core(TM) i7-13700K @ 3.40 GHz with 128 GB. 

ABD SHA `9dbd02e8868712b631406fa803d2fe449d4754e1`. RigidIPC SHA `dc8bc87c515bfe16b0c1ff74ccdb36f1e09ab46b`

# Peg Insertion 

## ABD
Time: 3.3278s for 150 iterations 

### Command line 
```
E:\Github\Inukshuk\build\Release>abd_sim.exe --ngui -o output --num-steps 150 -i ..\..\inputs\peg\peg_insertion.json
100%|██████████████████████████████████████████████████████████████████████| 150/150 (3.3s/3.3s)
Simulation finished (total_runtime=3.3278s average_fps=45.0748)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.001,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 1e-2,
        "minimum_separation_distance": 0.0
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.3,
        "gravity": [0, -1e3, 0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "peg\\0_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 4e-3
        },
        {
            "mesh": "peg\\1_mm.obj",
            "position": [0.0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 4e-3
        }],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "0_mm",
                "body_point": [0.0, 0.0, 1.0]
            },
            {
                "type": "pin_world",
                "body_name": "0_mm",
                "body_point": [1.0, 0.0, 0.0]
            },
            {
                "type": "pin_world",
                "body_name": "0_mm",
                "body_point": [0.0, 1.0, 0.0]
            }
        ]
    }
}
```



### RigidIPC

Time: 16.5769s for 150 iterations 

### Command line 
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe --ngui -o output --num-steps 150 -i ..\..\inputs\peg\peg_insertion.json
100%|██████████████████████████████████████████████████████████████████████| 150/150 (16.6s/16.6s)
Simulation finished (total_runtime=16.5769s average_fps=9.04876)
```

### JSON File (No change)
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.001,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 1e-2,
        "minimum_separation_distance": 0.0
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.3,
        "gravity": [0, -1e3, 0],        
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "peg\\0_mm.obj",
            "is_dof_fixed": true,
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 4e-3
        },
        {
            "mesh": "peg\\1_mm.obj",
            "position": [0.0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 4e-3
        }]
    }
}

```


# USB

## ABD
Time: 4.01643s for 50 iterations 

### Command line
```
E:\Github\Inukshuk\build\Release>abd_sim.exe -o output --num-steps 50 -i ..\..\inputs\usb_test_01\usb.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 50/50 (4.0s/4.0s)
Simulation finished (total_runtime=4.01643s average_fps=12.4489)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 1e-5,
        "minimum_separation_distance": 0.0
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-5
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0, -1.0, 0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "usb_test_01\\usb_socket.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 1000
        },
        {
            "mesh": "usb_test_01\\usb_male.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 1000
        }],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "usb_socket",
                "body_point": [0.0, 0.0, 1.0]
            },
            {
                "type": "pin_world",
                "body_name": "usb_socket",
                "body_point": [1.0, 0.0, 0.0]
            },
            {
                "type": "pin_world",
                "body_name": "usb_socket",
                "body_point": [0.0, 1.0, 0.0]
            }
        ]
    }
}
```


## RigidIPC
Time: 13.1s for 50 iterations 

### Command line
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe -o output --num-steps 50 -i ..\..\inputs\usb_test_01\usb.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 50/50 (13.1s/13.1s)
Simulation finished (total_runtime=13.0569s average_fps=3.82938)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 1e-5,
        "minimum_separation_distance": 0.0
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-5
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0, -1.0, 0],
        "rigid_bodies": [{
            "mesh": "E:\\Github\\rigid-ipc\\inputs\\usb_test_01\\usb_male_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 1000
        }, {
            "mesh": "E:\\Github\\rigid-ipc\\inputs\\usb_test_01\\usb_socket_mm.obj",
            "is_dof_fixed": true,
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 1000
        }]
    }
}
```



# Gear Offset 

## ABD
Time: 298.745s for 50 iterations 

### Command line 
```
E:\Github\Inukshuk\build\Release>abd_sim.exe -o output --num-steps 50 -i ..\..\inputs\gear_assembly_offset\gear_offset.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 50/50 (298.7s/298.7s)
Simulation finished (total_runtime=298.745s average_fps=0.167367)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 5e-3,
        "minimum_separation_distance": 1e-4
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0, -1e3, 0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "gear_assembly_offset\\3_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\0_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density":7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\2_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        }],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [0.0, 0.0, 1.0]
            },
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [1.0, 0.0, 0.0]
            },
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [0.0, 1.0, 0.0]
            }
        ]
    }
}
```



## RigidIPC
Time: 2337.6s for 500 iterations 

### Command line 
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe -o output --num-steps 500 -i ..\..\inputs\gear_assembly_offset\gear_offset.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 500/500 (2337.6s/2337.6s)
Simulation finished (total_runtime=2337.55s average_fps=0.213899)
```

### JSON File (Time step is 1/10)
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.001,
    "max_time": 0.5,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 5e-3,
        "minimum_separation_distance": 1e-4
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0, -1e3, 0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "gear_assembly_offset\\3_mm.obj",
            "is_dof_fixed": true,
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\0_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density":7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\2_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        }]
    }
}
```





# Gear Offset Rotating

## ABD
Time: 298.745s for 50 iterations 

### Command line 
```
E:\Github\Inukshuk\build\Release>abd_sim.exe -o output --num-steps 150 -i ..\..\inputs\gear_assembly_offset\gear_offset_rotating.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 150/150 (320.0s/320.0s)
Simulation finished (total_runtime=320.003s average_fps=0.468746)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 1.5,
    "distance_barrier_constraint": {        
        "initial_barrier_activation_distance": 5e-3,
        "minimum_separation_distance": 1e-4
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0.0, 0.0, 0.0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "gear_assembly_offset\\3_mm.obj",
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\1_mm.obj",
            "position": [0.0, -39.5, 0.0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3,
            "torque": [0.0, 0.0, 200000.0]
        },
        {
            "mesh": "gear_assembly_offset\\2_mm.obj",
            "position": [0, -25.0, 0.0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        }],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [0.0, 0.0, 1.0]
            },
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [1.0, 0.0, 0.0]
            },
            {
                "type": "pin_world",
                "body_name": "3_mm",
                "body_point": [0.0, 1.0, 0.0]
            }
        ]
    }
}
```



## RigidIPC
Time: 2337.6s for 500 iterations 

### Command line 
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe -o output --num-steps 150 -i ..\..\inputs\gear_assembly_offset\gear_offset_rotating.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 150/150 (5813.7s/5813.7s)
Simulation finished (total_runtime=5813.67s average_fps=0.0258013)
```

### JSON File
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 1.5,
    "distance_barrier_constraint": {        
        "initial_barrier_activation_distance": 5e-3,
        "minimum_separation_distance": 1e-4
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-3
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": -1,
        "coefficient_friction": 0.0,
        "gravity": [0.0, 0.0, 0.0],
        "orthogonality_stiffness": 1e9,
        "rigid_bodies": [{
            "mesh": "gear_assembly_offset\\3_mm.obj",
			"is_dof_fixed": true,
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        },
        {
            "mesh": "gear_assembly_offset\\1_mm.obj",
            "position": [0.0, -39.5, 0.0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3,
            "torque": [0.0, 200000.0, 0.0]
        },
        {
            "mesh": "gear_assembly_offset\\2_mm.obj",
            "position": [0, -25.0, 0.0],
            "rotation": [0, 0, 0],
            "density": 7.8e-3
        }]
    }
}
```






# BNC Crimp 

## ABD
Time: 43.8s for 23 iterations

### Command line 
```
E:\Github\Inukshuk\build\Release>abd_sim.exe -o output --num-steps 23 -i ..\..\inputs\bnc_crimp\bnc.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 23/23 (43.8s/43.8s)
Simulation finished (total_runtime=43.7855s average_fps=0.525288)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 1.0,
    "distance_barrier_constraint": {        
        "initial_barrier_activation_distance": 1e-5
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-5
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": 0.0,
        "coefficient_friction": 0.0,
        "gravity": [
            0.0,
            0.0,
            0.0
        ],
        "rigid_bodies": [
            {
                "mesh": "bnc_crimp\\0.obj",
                "position": [
                    0,
                    0,
                    0
                ],
                "force": [
                    0,
                    -0.009210452,
                    0
                ],                
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            },
            {
                "mesh": "bnc_crimp\\1.obj",                
                "position": [
                    0,
                    0,
                    0
                ],
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            }
        ],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "1",
                "body_point": [
                    0.0,
                    0.0,
                    1.0
                ]
            },
            {
                "type": "pin_world",
                "body_name": "1",
                "body_point": [
                    1.0,
                    0.0,
                    0.0
                ]
            },
            {
                "type": "pin_world",
                "body_name": "1",
                "body_point": [
                    0.0,
                    1.0,
                    0.0
                ]
            }
        ]
    }
}
```



## RigidIPC
Time: 109.0s for 228 iterations

### Command line 
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe  -o output --num-steps 228 -i ..\..\inputs\bnc_crimp\bnc.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 228/228 (109.0s/109.0s)
Simulation finished (total_runtime=109.042s average_fps=2.09094)
```

### JSON File (Time step is 1/10)
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.001,
    "max_time": 1.0,
    "distance_barrier_constraint": {
        "barrier_type": "ipc",
        "initial_barrier_activation_distance": 1e-5
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-5
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": 0.0,
        "coefficient_friction": 0.0,
        "gravity": [0.0 , 0.0, 0.0],
        "rigid_bodies": [{
            "mesh": "bnc_crimp\\0.obj",
            "position": [0, 0, 0],
            "force": [0, -0.009210452, 0],
            "is_dof_fixed": false,
            "rotation": [0, 0, 0],
            "density": 7800
        }, {
            "mesh": "bnc_crimp\\1.obj",
            "is_dof_fixed": true,
            "position": [0, 0, 0],
            "rotation": [0, 0, 0],
            "density": 7800
        }]
    }
}
```




# DSUB-25pin

## ABD

Time: 2.91768s for 12 iterations

### Command line 
```
E:\Github\Inukshuk\build\Release>abd_sim.exe -o output --num-steps 12 -i ..\..\inputs\dsub_25pin\dsub.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 12/12 (2.9s/2.9s)
Simulation finished (total_runtime=2.91768s average_fps=4.11286)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.01,
    "max_time": 1,
    "distance_barrier_constraint": {
        "initial_barrier_activation_distance": 1e-5
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-4
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": 0.0,
        "coefficient_friction": 0.0,
        "gravity": [
            0,
            0.0,
            0
        ],
        "rigid_bodies": [
            {
                "mesh": "dsub_25pin\\1.obj",
                "position": [
                    0,
                    0,
                    0
                ],
                "force": [
                    0.0,
                    -0.034260444,
                    0.0
                ],
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            },
            {
                "mesh": "dsub_25pin\\0.obj",                
                "position": [
                    0,
                    0,
                    0
                ],
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            }
        ],
        "linear_constraints": [
            {
                "type": "pin_world",
                "body_name": "0",
                "body_point": [
                    0.0,
                    0.0,
                    1.0
                ]
            },
            {
                "type": "pin_world",
                "body_name": "0",
                "body_point": [
                    1.0,
                    0.0,
                    0.0
                ]
            },
            {
                "type": "pin_world",
                "body_name": "0",
                "body_point": [
                    0.0,
                    1.0,
                    0.0
                ]
            }
        ]
    }
}
```


## RigidIPC
Time: 99.1393s for 123 iterations

### Command line 
```
E:\Github\temp\Inukshuk\build\Release>rigid_ipc_sim.exe  -o output --num-steps 123 -i ..\..\inputs\dsub_25pin\dsub.json --ngui
100%|██████████████████████████████████████████████████████████████████████| 123/123 (99.1s/99.1s)
Simulation finished (total_runtime=99.1393s average_fps=1.24068)
```

### JSON File 
```json 
{
    "scene_type": "distance_barrier_rb_problem",
    "solver": "ipc_solver",
    "timestep": 0.001,
    "max_time": 1,
    "distance_barrier_constraint": {
        "initial_barrier_activation_distance": 1e-5
    },
    "ipc_solver": {
        "velocity_conv_tol": 1e-4
    },
    "friction_constraints": {
        "iterations": -1
    },
    "rigid_body_problem": {
        "coefficient_restitution": 0.0,
        "coefficient_friction": 0.0,
        "gravity": [
            0,
            0.0,
            0
        ],
        "rigid_bodies": [
            {
                "mesh": "dsub_25pin\\1.obj",
                "position": [
                    0,
                    0,
                    0
                ],
                "force": [
                    0.0,
                    -0.034260444,
                    0.0
                ],
                "is_dof_fixed": false,
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            },
            {
                "mesh": "dsub_25pin\\0.obj",
                "is_dof_fixed": true,
                "position": [
                    0,
                    0,
                    0
                ],
                "rotation": [
                    0,
                    0,
                    0
                ],
                "density": 7800
            }
        ]
    }
}
```
