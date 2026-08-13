"""Pure navigation status derivation shared by the HTTP API and tests."""


def derive_navigation_state(*, target_active, planner_active, obstacle,
                            outcome, has_route=False):
    if target_active:
        if obstacle is True:
            return {'code': 'blocked', 'label': '前方有障碍'}
        if planner_active:
            return {'code': 'planning', 'label': '实时规划中'}
        return {'code': 'waiting_path', 'label': '等待可行路径'}

    if outcome == 'completed':
        return {
            'code': 'completed',
            'label': '路线已完成' if has_route else '已到达目标',
        }
    if outcome == 'stopped':
        return {'code': 'stopped', 'label': '导航已停止'}
    return {'code': 'idle', 'label': '未设置目标'}
