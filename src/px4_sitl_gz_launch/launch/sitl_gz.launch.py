#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Launch PX4 SITL (POSIX + gz) with explicit environment and vehicle model.
#
# Typical usage (after colcon build --packages-select px4_sitl_gz_launch):
#   export PX4_SOURCE_DIR=/path/to/PX4-Autopilot
#   ros2 launch px4_sitl_gz_launch sitl_gz.launch.py \\
#     px4_sim_model:=gz_x500 px4_gz_world:=default \\
#     spawn_x:=2 spawn_y:=1 spawn_z:=0.5
#   # 或完整位姿 (米、弧度): px4_gz_model_pose:='1,2,0.5,0,0,1.57'
#
# spawn_gz_first:=true (默认)先在本 launch 里启动 gz sim, 再延迟启动 px4 并设置
# PX4_GZ_STANDALONE=1, 使 gz_bridge 内重试连接, 避免 px4 内置「先起 gz 再起 bridge」的竞态超时。
#
# 自定义 Gazebo 无人机模型 (spawn): 
#   - 目录布局: <某搜索路径>/<模型名>/model.sdf (可参考 Tools/simulation/gz/models/x500/)。
#   - 将该搜索路径加入 extra_gz_resource_path (其父目录需在 GZ_SIM_RESOURCE_PATH 中)。
#   - px4_sim_model 设为 gz_<模型名>, 且 ROMFS 须有对应机架 init.d-posix/airframes/*_gz_<模型名>
#      (可复制 4001_gz_x500 改参_mixer; 否则 PX4 控制与仿真不一致)。
# 场景中已有模型 (attach): 在 gz 里先放好飞机, 再 px4_gz_model_name:=实体名 (见 ROMFS px4-rc.simulator)。
#

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration


def _pkg_sim_paths():
    """包内仿真资源路径 (colcon install 后的 share/px4_sitl_gz_launch/{models,worlds})。"""
    pkg_share = get_package_share_directory('px4_sitl_gz_launch')
    return (
        os.path.join(pkg_share, 'worlds'),
        os.path.join(pkg_share, 'models'),
    )


def _merge_gz_resource_env(env: dict, px4_src: str, worlds_dir_arg: str) -> None:
    """Mirror rootfs/gz_env.sh so gz_bridge can resolve model.sdf (spawn via EntityFactory)."""
    gz_models = os.path.join(px4_src, 'Tools', 'simulation', 'gz', 'models')
    if worlds_dir_arg.strip():
        gz_worlds = os.path.abspath(os.path.expanduser(worlds_dir_arg.strip()))
    else:
        gz_worlds = os.path.join(px4_src, 'Tools', 'simulation', 'gz', 'worlds')
    env['PX4_GZ_MODELS'] = gz_models
    env['PX4_GZ_WORLDS'] = gz_worlds
    cur = env.get('GZ_SIM_RESOURCE_PATH', '')
    env['GZ_SIM_RESOURCE_PATH'] = f'{cur}:{gz_models}:{gz_worlds}' if cur else f'{gz_models}:{gz_worlds}'


def _resolve_px4_paths(px4_src: str, sitl_build: str, px4_binary_override: str):
    """Return (path to px4 executable, rootfs cwd)."""
    if px4_binary_override:
        bin_px4 = os.path.abspath(os.path.expanduser(px4_binary_override))
        rootfs = os.path.join(px4_src, 'build', sitl_build, 'rootfs')
        return bin_px4, rootfs

    rootfs = os.path.join(px4_src, 'build', sitl_build, 'rootfs')
    bin_px4 = os.path.join(px4_src, 'build', sitl_build, 'bin', 'px4')
    return bin_px4, rootfs


def _world_sdf_path(px4_src: str, worlds_dir_arg: str, world_basename: str) -> str:
    if worlds_dir_arg.strip():
        gz_worlds = os.path.abspath(os.path.expanduser(worlds_dir_arg.strip()))
    else:
        gz_worlds = os.path.join(px4_src, 'Tools', 'simulation', 'gz', 'worlds')
    return os.path.join(gz_worlds, f'{world_basename}.sdf')


def _apply_spawn_pose(env: dict, context) -> None:
    """Map ROMFS px4-rc.simulator: PX4_GZ_MODEL_POSE → x,y,z,roll,pitch,yaw (m, rad)."""
    pose_full = LaunchConfiguration('px4_gz_model_pose').perform(context).strip()
    sx = LaunchConfiguration('spawn_x').perform(context).strip()
    sy = LaunchConfiguration('spawn_y').perform(context).strip()
    sz = LaunchConfiguration('spawn_z').perform(context).strip()
    sr = LaunchConfiguration('spawn_roll').perform(context).strip()
    sp = LaunchConfiguration('spawn_pitch').perform(context).strip()
    syaw = LaunchConfiguration('spawn_yaw').perform(context).strip()

    def _z(v: str) -> str:
        return v if v else '0'

    if pose_full:
        env['PX4_GZ_MODEL_POSE'] = pose_full
        return

    if sx or sy or sz or sr or sp or syaw:
        env['PX4_GZ_MODEL_POSE'] = ','.join(
            [_z(sx), _z(sy), _z(sz), _z(sr), _z(sp), _z(syaw)]
        )


def _launch_setup(context, *_args, **_kwargs):
    px4_src = LaunchConfiguration('px4_src').perform(context).strip()
    if not px4_src:
        px4_src = os.environ.get('PX4_SOURCE_DIR', '').strip()
    if not px4_src:
        raise RuntimeError(
            'Set launch argument px4_src to the PX4 repository root, or export PX4_SOURCE_DIR.'
        )
    px4_src = os.path.abspath(os.path.expanduser(px4_src))

    sitl_build = LaunchConfiguration('sitl_build').perform(context).strip()
    px4_binary_override = LaunchConfiguration('px4_binary').perform(context).strip()

    bin_px4, rootfs = _resolve_px4_paths(px4_src, sitl_build, px4_binary_override)

    spawn_gz_first = LaunchConfiguration('spawn_gz_first').perform(context).lower() == 'true'
    try:
        delay_sec = float(LaunchConfiguration('gz_ready_delay_sec').perform(context).strip() or '5.0')
    except ValueError:
        delay_sec = 5.0

    env = os.environ.copy()

    model = LaunchConfiguration('px4_sim_model').perform(context).strip()
    if model:
        env['PX4_SIM_MODEL'] = model

    gz_model_name = LaunchConfiguration('px4_gz_model_name').perform(context).strip()
    if gz_model_name:
        env['PX4_GZ_MODEL_NAME'] = gz_model_name

    raw_world = LaunchConfiguration('px4_gz_world').perform(context).strip()
    world = raw_world
    if world.lower().endswith('.sdf'):
        world = world[:-4]
    effective_world = world if world else 'default'
    env['PX4_GZ_WORLD'] = effective_world

    autostart = LaunchConfiguration('px4_sys_autostart').perform(context).strip()
    if autostart:
        env['PX4_SYS_AUTOSTART'] = autostart

    user_standalone = LaunchConfiguration('gz_standalone').perform(context).lower() == 'true'

    headless = LaunchConfiguration('headless').perform(context).lower()
    if headless == 'true':
        env['HEADLESS'] = '1'
    elif headless == 'false' and 'HEADLESS' in env:
        del env['HEADLESS']

    render_engine = LaunchConfiguration('gz_render_engine').perform(context).strip()
    if render_engine:
        env['PX4_GZ_SIM_RENDER_ENGINE'] = render_engine
    elif 'PX4_GZ_SIM_RENDER_ENGINE' in env:
        del env['PX4_GZ_SIM_RENDER_ENGINE']

    extra_res = LaunchConfiguration('extra_gz_resource_path').perform(context).strip()
    if extra_res:
        base = env.get('GZ_SIM_RESOURCE_PATH', '')
        sep = ':' if base else ''
        env['GZ_SIM_RESOURCE_PATH'] = extra_res + sep + base

    worlds_dir = LaunchConfiguration('px4_gz_worlds_dir').perform(context).strip()
    _merge_gz_resource_env(env, px4_src, worlds_dir)

    instance = LaunchConfiguration('px4_instance').perform(context).strip()
    if instance:
        env['PX4_INSTANCE'] = instance

    _apply_spawn_pose(env, context)

    warn_msgs = []
    if not os.path.isfile(bin_px4):
        warn_msgs.append(f'px4 binary not found: {bin_px4} (build px4_sitl first)')
    if not os.path.isdir(rootfs):
        warn_msgs.append(f'rootfs directory missing: {rootfs}')
    if not shutil.which('gz'):
        warn_msgs.append('executable "gz" not found on PATH (install Gazebo Harmonic/Garden)')

    world_sdf = _world_sdf_path(px4_src, worlds_dir, effective_world)
    if spawn_gz_first and not os.path.isfile(world_sdf):
        warn_msgs.append(f'world file missing: {world_sdf}')

    actions = []
    for m in warn_msgs:
        actions.append(LogInfo(msg=f'[px4_sitl_gz_launch] WARNING: {m}'))

    if raw_world and raw_world.lower().endswith('.sdf'):
        actions.append(
            LogInfo(
                msg=(
                    f'[px4_sitl_gz_launch] px4_gz_world should be the basename without .sdf; '
                    f'using PX4_GZ_WORLD={effective_world}. '
                    f'gz_bridge uses /world/<PX4_GZ_WORLD>/create — must match '
                    f'<world name="..."> in your .sdf.'
                )
            )
        )

    # ----- bundled: px4 starts gz internally (may race gz_bridge 1s timeout) -----
    if not spawn_gz_first:
        if user_standalone:
            env['PX4_GZ_STANDALONE'] = '1'
        elif 'PX4_GZ_STANDALONE' in env:
            del env['PX4_GZ_STANDALONE']

        actions.append(
            ExecuteProcess(
                cmd=[bin_px4],
                cwd=rootfs,
                env=env,
                output='screen',
                shell=False,
            )
        )
        return actions

    # ----- two-step: gz sim here, then px4 with STANDALONE=1 (gz_bridge retries) -----
    if user_standalone:
        actions.append(
            LogInfo(
                msg=(
                    '[px4_sitl_gz_launch] spawn_gz_first:=true 与 gz_standalone:=true 同时启用: '
                    '假定 gz 已由外部启动, 本 launch 不再启动 gz sim。'
                )
            )
        )
        px4_env = dict(env)
        px4_env['PX4_GZ_STANDALONE'] = '1'
        actions.append(
            TimerAction(
                period=delay_sec,
                actions=[
                    ExecuteProcess(
                        cmd=[bin_px4],
                        cwd=rootfs,
                        env=px4_env,
                        output='screen',
                        shell=False,
                    )
                ],
            )
        )
        return actions

    gz_server_cmd = ['gz', 'sim']
    if render_engine:
        gz_server_cmd.extend(['--render-engine', render_engine])
    gz_server_cmd.extend(['--verbose=1', '-r', '-s', world_sdf])

    actions.append(
        LogInfo(
            msg=(
                f'[px4_sitl_gz_launch] spawn_gz_first: starting gz, then px4 after {delay_sec}s '
                f'(PX4_GZ_STANDALONE=1 for gz_bridge retry). World file: {world_sdf}'
            )
        )
    )

    actions.append(
        ExecuteProcess(
            cmd=gz_server_cmd,
            env=dict(env),
            output='screen',
            shell=False,
        )
    )

    if headless != 'true':
        actions.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                env=dict(env),
                output='screen',
                shell=False,
            )
        )

    px4_env = dict(env)
    px4_env['PX4_GZ_STANDALONE'] = '1'
    if 'HEADLESS' in px4_env:
        del px4_env['HEADLESS']

    actions.append(
        TimerAction(
            period=delay_sec,
            actions=[
                ExecuteProcess(
                    cmd=[bin_px4],
                    cwd=rootfs,
                    env=px4_env,
                    output='screen',
                    shell=False,
                )
            ],
        )
    )
    return actions


def generate_launch_description():
    default_worlds_dir, default_models_dir = _pkg_sim_paths()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_src',
                default_value=os.environ.get('PX4_SOURCE_DIR', ''),
                description='PX4 源码根目录 (包含 build/ 与 Tools/simulation/gz)。可用环境变量 PX4_SOURCE_DIR。',
            ),
            DeclareLaunchArgument(
                'sitl_build',
                default_value='px4_sitl_default',
                description='构建目录名: build/<sitl_build>/bin/px4',
            ),
            DeclareLaunchArgument(
                'px4_binary',
                default_value='',
                description='若指定则直接使用该 px4 可执行文件路径。',
            ),
            DeclareLaunchArgument(
                'px4_sim_model',
                default_value='drone260',
                description='仿真机型: PX4_SIM_MODEL=gz_<名>, 对应 gz_bridge 加载 <名>/model.sdf; '
                '须在 GZ_SIM_RESOURCE_PATH 内可解析; 自定义模型见文件头注释并配合 extra_gz_resource_path; '
                '还需 ROMFS 机架脚本 *_gz_<名>。',
            ),
            DeclareLaunchArgument(
                'px4_gz_model_name',
                default_value='',
                description='可选: 场景中已存在模型时设置 PX4_GZ_MODEL_NAME, gz_bridge 附着到该实体而非 spawn '
                ' (与 px4-rc.simulator 一致; 此时通常不再走 gz_<名>/model.sdf 生成)。',
            ),
            DeclareLaunchArgument(
                'px4_gz_world',
                default_value='custom',
                # default_value='room',
                description='世界 basename (无 .sdf); 必须与 SDF 内 <world name="…"> 一致。',
            ),
            DeclareLaunchArgument(
                'px4_gz_worlds_dir',
                default_value=default_worlds_dir,
                description='覆盖 worlds 目录; 默认包内 share/.../worlds; 空则 <px4_src>/Tools/simulation/gz/worlds。',
            ),
            DeclareLaunchArgument(
                'px4_gz_model_pose',
                default_value='',
                description='生成无人机初始位姿 (优先于 spawn_*): x,y,z,roll,pitch,yaw, 米与弧度; '
                '也可用空格分隔, px4 启动脚本会规范化。',
            ),
            DeclareLaunchArgument(
                'spawn_x',
                default_value='',
                description='初始位置 X (米); 与 spawn_y/z 等组合为位姿, 未填的分量视为 0。',
            ),
            DeclareLaunchArgument(
                'spawn_y',
                default_value='',
                description='初始位置 Y (米)。',
            ),
            DeclareLaunchArgument(
                'spawn_z',
                default_value='',
                description='初始高度 Z (米); 仅改高度可 spawn_z:=1.5。',
            ),
            DeclareLaunchArgument(
                'spawn_roll',
                default_value='',
                description='初始横滚 (弧度)。',
            ),
            DeclareLaunchArgument(
                'spawn_pitch',
                default_value='',
                description='初始俯仰 (弧度)。',
            ),
            DeclareLaunchArgument(
                'spawn_yaw',
                default_value='',
                description='初始偏航 (弧度)。',
            ),
            DeclareLaunchArgument(
                'px4_sys_autostart',
                default_value='',
                description='可选: PX4_SYS_AUTOSTART。',
            ),
            DeclareLaunchArgument(
                'gz_standalone',
                default_value='false',
                description='true: 不启动 gz (仅 PX4)。若 spawn_gz_first:=true, 则不再拉起 gz sim。',
            ),
            DeclareLaunchArgument(
                'spawn_gz_first',
                default_value='true',
                description='true: 在本 launch 内先 gz sim -s, 再延迟启动 px4 并 STANDALONE=1, 避免 gz_bridge 超时。',
            ),
            DeclareLaunchArgument(
                'gz_ready_delay_sec',
                default_value='5.0',
                description='spawn_gz_first 时, gz sim 启动后等待多少秒再启动 px4。',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='true: 不启动 gz GUI (gz sim -g)。',
            ),
            DeclareLaunchArgument(
                'gz_render_engine',
                default_value='',
                description='可选: gz sim --render-engine。',
            ),
            DeclareLaunchArgument(
                'extra_gz_resource_path',
                default_value=default_models_dir,
                description='前置追加到 GZ_SIM_RESOURCE_PATH (冒号分隔); 默认包内 share/.../models。'
                '自定义模型目录若为你的_models/my_quad/model.sdf, 则填「包含 my_quad 的父路径」: your_models。',
            ),
            DeclareLaunchArgument(
                'px4_instance',
                default_value='',
                description='可选: PX4_INSTANCE。',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
