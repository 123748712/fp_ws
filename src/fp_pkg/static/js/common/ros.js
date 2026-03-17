function fn_connectRos() {
    var ros = new ROSLIB.Ros({url : 'ws://192.168.0.57:9090'});
    ros.on('connection', function() { console.log('Connected to websocket server.'); });
    ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); });
    ros.on('close', function() { console.log('Connection to websocket server closed.'); });

    return ros;
}

function fn_getTopic(topicType) {
    var topic;
    if(topicType == "pose") {
        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/amcl_pose',
            messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
            latch: true
        });
    } else if(topicType == "scan") {
        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/scan', // 이거는 추후 실제 터틒봇 스캔 토픽으로 변경
            messageType: 'sensor_msgs/msg/LaserScan',
            queue_size: 1,
            throttle_rate: 100,
            reliability: 'best_effort'
        });
    } else if(topicType == "initial") {
        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/initialpose',
            messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
        });
    } else if(topicType == "goal") {
        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/goal_pose',
            messageType: 'geometry_msgs/msg/PoseStamped'
        });
    } else if(topicType == "cmd_vel") {
        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        });
    } else if(topicType == "path") {
        topic = new ROSLIB.Topic({
            ros : ros,
            name : '/planned_path',
            messageType : 'nav_msgs/msg/Path'
        });
    } else if(topicType == "battery") {
        topic = new ROSLIB.Topic({
            ros : ros,
            name : '/battery_state',
            messageType : 'sensor_msgs/msg/BatteryState'
        });
    } else if(topicType == "odom") {
        topic = new ROSLIB.Topic({
            ros : ros,
            name : '/odom',
            messageType : 'nav_msgs/msg/Odometry'
        });
    }
    return topic;
}