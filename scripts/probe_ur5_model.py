"""探测组装后的 UR5e + 2F-85 模型参数，供适配决策使用（只读，不改模型）。"""

import argparse
from pathlib import Path

import mujoco


def name(model, obj, index):
    return mujoco.mj_id2name(model, obj, index)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    args = parser.parse_args()

    path = Path(args.model).resolve()
    model = mujoco.MjModel.from_xml_path(str(path))
    print("model", path)
    print("nq=%d nv=%d nu=%d nbody=%d ngeom=%d nsite=%d nmocap=%d"
          % (model.nq, model.nv, model.nu, model.nbody, model.ngeom,
             model.nsite, model.nmocap))
    print("timestep=%s integrator=%s cone=%s impratio=%s"
          % (model.opt.timestep, model.opt.integrator, model.opt.cone,
             model.opt.impratio))

    print("\n--- actuators ---")
    for index in range(model.nu):
        print(
            "%-28s trn=%d gain=%s bias=%s forcerange=%s ctrlrange=%s"
            % (
                name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index),
                int(model.actuator_trntype[index]),
                [round(float(v), 6) for v in model.actuator_gainprm[index][:3]],
                [round(float(v), 6) for v in model.actuator_biasprm[index][:3]],
                [float(v) for v in model.actuator_forcerange[index]],
                [float(v) for v in model.actuator_ctrlrange[index]],
            )
        )

    print("\n--- joints ---")
    for index in range(model.njnt):
        jname = name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        joint_range = [round(float(v), 6) for v in model.jnt_range[index]]
        print(
            "%-32s type=%d qposadr=%d dofadr=%d range=%s stiffness=%s damping=%s"
            % (
                jname,
                int(model.jnt_type[index]),
                int(model.jnt_qposadr[index]),
                int(model.jnt_dofadr[index]),
                joint_range,
                round(float(model.jnt_stiffness[index]), 6),
                round(float(model.dof_damping[int(model.jnt_dofadr[index])]), 6),
            )
        )

    print("\n--- sites ---")
    for index in range(model.nsite):
        print("%-28s body=%s pos=%s quat=%s"
              % (
                  name(model, mujoco.mjtObj.mjOBJ_SITE, index),
                  name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[index])),
                  [round(float(v), 6) for v in model.site_pos[index]],
                  [round(float(v), 6) for v in model.site_quat[index]],
              ))

    print("\n--- gripper bodies ---")
    for index in range(model.nbody):
        bname = name(model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
        if "pad" in bname or "finger" in bname:
            print("%-30s pos=%s"
                  % (bname, [round(float(v), 6) for v in model.body_pos[index]]))

    print("\n--- gripper pad geoms ---")
    for index in range(model.ngeom):
        gname = name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or ""
        if "pad" in gname:
            print("%-30s type=%d size=%s friction=%s contype=%d conaffinity=%d"
                  % (
                      gname,
                      int(model.geom_type[index]),
                      [round(float(v), 6) for v in model.geom_size[index]],
                      [round(float(v), 4) for v in model.geom_friction[index]],
                      int(model.geom_contype[index]),
                      int(model.geom_conaffinity[index]),
                  ))

    print("\n--- tendons / equality ---")
    for index in range(model.ntendon):
        print("tendon %-20s type=%d"
              % (name(model, mujoco.mjtObj.mjOBJ_TENDON, index), int(model.tendon_type[index])))
    print("neq=%d" % model.neq)
    for index in range(model.neq):
        print("eq %d type=%d active=%s"
              % (index, int(model.eq_type[index]), bool(model.eq_active0[index])))


if __name__ == "__main__":
    main()
