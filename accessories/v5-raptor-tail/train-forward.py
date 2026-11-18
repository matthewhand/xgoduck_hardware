"""Register and launch a local mjlab PPO experiment for the passive V5 tail.

Run inside LuwuDynamics/xgoduck_rl at a4c78aac (mjlab 1.3.0).
Run with --check to construct the task and compile its robot before PPO.
"""
import sys

from tail_physics import PITCH, YAW, configure, read_profile

ID = 'Mjlab-Velocity-Forward-XgoDuck-PassiveTail'
BASE = 'Mjlab-Velocity-Flat-XgoDuck'


def robot_spec():
    from mjlab_microduck.robot.xgoduck_constants import get_walk_spec
    spec = get_walk_spec()
    configure(spec,read_profile(),'passive')
    return spec


def flight_reward(env):
    # Short unsupported intervals only. A tail-drag or a body bouncing along
    # the floor cannot count as flight simply because both feet are lifted.
    robot = env.scene['robot'].data
    feet = env.scene['feet_ground_contact'].data.current_air_time
    floor = env.scene['whole_duck_ground'].data.found
    airborne = feet.min(dim=1).values
    quat = robot.root_link_quat_w
    upright = 1 - 2 * (quat[:,1].square()+quat[:,2].square())
    return ((airborne>=.020) & (airborne<=.12) & (floor.sum(dim=1)==0) &
            (robot.root_link_lin_vel_b[:,0]>=.30) & (upright>.92)).float()


def register():
    import mjlab_microduck.tasks  # registers the pinned stock velocity task
    from mjlab.managers.reward_manager import RewardTermCfg
    from mjlab.sensor import ContactMatch, ContactSensorCfg
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls, register_mjlab_task
    configs=[]
    for play in (False,True):
        cfg = load_env_cfg(BASE,play=play)
        robot = cfg.scene.entities['robot']
        robot.spec_fn = robot_spec
        for hinge in (PITCH,YAW):
            robot.init_state.joint_pos[hinge]=0.0
        # The stock XgoDuck BAM actuators and joint observations exclude all
        # names beginning with passive_, preserving the 14-action contract.
        cfg.scene.sensors = (*cfg.scene.sensors,ContactSensorCfg(
            name='whole_duck_ground',primary=ContactMatch(mode='subtree',pattern='trunk_base',entity='robot'),
            secondary=ContactMatch(mode='body',pattern='terrain'),fields=('found',),reduce='maxforce',num_slots=1))
        cfg.rewards['forward_flight'] = RewardTermCfg(func=flight_reward,weight=1.5)
        cmd = cfg.commands['twist']
        cmd.independent_command_fraction=0
        cmd.rel_standing_envs=0
        cmd.rel_forward_envs=cmd.rel_turn_in_place_envs=cmd.rel_backward_envs=0
        cmd.rel_heading_envs=cmd.rel_world_envs=0
        cmd.heading_command=False
        cmd.ranges.heading=None
        cmd.ranges.lin_vel_x=(.35,.55)
        cmd.ranges.lin_vel_y=cmd.ranges.ang_vel_z=(0.,0.)
        cfg.curriculum.pop('standing_envs',None)
        cfg.curriculum.pop('command_vel',None)
        cfg.commands['head_pose'].ranges=((0.,0.),)*4
        cfg.commands['head_pose'].zero_command_prob=1.
        cfg.curriculum.pop('head_pose_range',None)
        configs.append(cfg)
    rl = load_rl_cfg(BASE)
    rl.experiment_name='xgoduck_passive_tail_forward'
    register_mjlab_task(task_id=ID,env_cfg=configs[0],play_env_cfg=configs[1],rl_cfg=rl,runner_cls=load_runner_cls(BASE))


def main():
    register()
    if sys.argv[1:]==['--check']:
        model=robot_spec().compile()
        assert model.nu==14
        for name in (PITCH,YAW):
            assert model.dof_armature[model.joint(name).dofadr[0]]==0
        print(ID, 'compiled', model.nq,'coordinates,',model.nu,'actuators')
        return
    from mjlab.scripts.train import main as upstream_train
    sys.argv.insert(1,ID)
    return upstream_train()


if __name__=='__main__':
    sys.exit(main())
