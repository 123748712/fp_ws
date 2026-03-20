// 메인 페이지 로딩 시, 설정 함수
function fn_init() {
    fn_getNodeType();
    fn_getMaps();
}

// 노드 생성 시, select의 노드 타입 조회 함수
function fn_getNodeType() {
    // 노드 생성 모달의 select option 비동기 조회
    $.ajax({
        url: '/node/getNodeType.do',
        type: 'GET',
        contentType: "application/json",
        success: function(data) {
            var html = '';
            var select = $("#nodeType");
            $.each(data, function(idx, item) {
                html += '<option value="' + item.code_id + '">';
                html += item.code_name;
                html += '</option>';
            });
           $("#nodeType").html(html);
        },
        error: function(e) {
            toastr.error('노드 조회 중 에러가 발생했습니다 !', e);
        }
    });
}

// 노드 생성/수정 시, 각기 다른 모달 출력 함수
function fn_openNodeModal(node) {
    if(node) { // 지도 위 노드 클릭 시
        $("#nodeTitle").text("노드 수정");
        $("#nodeXCoord").val(node.x);
        $("#nodeYCoord").val(node.y);
        $("#nodeType").val(node.nodeId);
        $("#deleteNodeBtn").show();
        $("editCoordBtn").show();
    } else { // 신규 생성 모드
        $("#nodeModalTitle").text("새 노드 생성");
        $("#nodeType").val("");
        $("#deleteNodeBtn").hide();
        $("editCoordBtn").hide();
    }
    $("#nodeModal").modal("show");
}

// 노드 수정/저장 함수
function fn_saveNode() {
    if($("#nodeType").val().startsWith("NODE")) {
        if (waypoints.length >= 4) {
            alert("최대 4개까지만 생성 가능합니다.");
            startPos = null;
            resetFlags();
            return;
        }
    }

    var data = {
        "node_id" : $("#nodeType").val(),
        "node_x_coord" : $("#nodeXCoord").val(),
        "node_y_coord" : $("#nodeYCoord").val(),
        "map_id" : $("#mapSelect").val()
    };
    $.ajax({
        url: '/node/saveNode.do',
        type: 'POST',
        data: JSON.stringify(data),
        contentType: "application/json",
        success: function(data) {
            if(data.result === "success") {
                toastr.success('노드 저장을 성공헸습니다 !', '노드 저장');
                fn_getNodes();
            } else {
                toastr.error('노드 저장 중 에러가 발생했습니다 !', '노드 저장');
            }
            $("#nodeModal").modal("hide");

             // 노드 생성 -> 콜릭 시, 클릭 좌표로 수정되지만 혹시 모르니 값 초기화
            $("#nodeXCoord").val("");
            $("#nodeYCoord").val("");
        },
        error: function(e) {
            toastr.error('노드 저장 중 에러가 발생했습니다 !', '노드 저장');
        }
    });
}

// 노드 전체 조회 함수
function fn_getNodes() {
    $.ajax({
        url: '/node/getNodes.do',
        type: 'POST',
        data: JSON.stringify({"map_id" : $("#mapSelect").val()}),
        contentType: "application/json",
        success: function(data) {
            fn_createNodes(data);
        },
        error: function(e) {
            toastr.error('노드 조회 중 에러가 발생했습니다 !', '노드 조회');
        }
    });
}

// 노드 삭제 함수
function fn_deleteNode() {
    var data = {
        "node_id" : $("#nodeType").val()
    };
    $.ajax({
        url: '/node/deleteNode.do',
        type: 'POST',
        data: JSON.stringify(data),
        contentType: "application/json",
        success: function(data) {
            if(data.result === "success") {
                toastr.success('노드 삭제를 성공헸습니다 !', '노드 삭제');
                fn_getNodes();
            } else {
                toastr.error('노드 삭제 중 에러가 발생했습니다 !', '노드 삭제');
            }
            $("#nodeModal").modal("hide");
            fn_getNodes();
        },
        error: function(e) {
            toastr.error('노드 삭제 중 에러가 발생했습니다 !', '노드 삭제');
        }
    });
}

var isInitializing = false;
function fn_getMaps() {
     $.ajax({
        url: '/map/getMaps.do',
        type: 'GET',
        contentType: "application/json",
        success: function(data) {
            var html = '';
            $.each(data, function(idx, item) {
                html += '<option ';
                if(item.active_yn === "Y") { // 서버에 저장된 사용 여부에 따라 선택
                    html += 'selected ';
                }
                html += 'value="' + item.map_seq + '">';
                html += item.map_name;
                html += '</option>';
            });
            isInitializing = true; // html을 넣게 되면 select change 발생되기에 발생 이벤트 막기
            $("#mapSelect").html(html);
            isInitializing = false;
            fn_initMap();
        },
        error: function(e) {
            toastr.error('맵 조회 중 에러가 발생했습니다 !', e);
        }
    });
}

// 맵 변경 함수
function fn_switchMap() {
    if (isInitializing) return;
    fn_stopSlamPoseTracking();
    fn_stopSlamMap();
    // 지도 변경 시 모든 버튼 이벤트 disabled
    $("#mapSelect, #poseBtn, #goalBtn, #nodeBtn").prop("disabled", true);

    toastr.info(
        "지도 전환 중입니다. 이 과정은 약 30초정도 진행됩니다.", 
        "지도 변경", 
        { 
            timeOut: 3000,        // 메시지가 사라지기 전까지의 시간 (30초)
            extendedTimeOut: 1000, // 마우스를 올렸을 때 추가로 유지되는 시간
            progressBar: true,      // 남은 시간을 시각적으로 보여줌 (권장)
            closeButton: true       // 사용자가 직접 끌 수 있게 버튼 추가
        }
    );

    if (poseTopic) poseTopic.unsubscribe();
    if (scanTopic) scanTopic.unsubscribe();
    if (laserContainer) laserContainer.removeAllChildren();
    if (nodeContainer) nodeContainer.removeAllChildren();
    if (robotContainer) robotContainer.visible = false;

    $.ajax({
        url: '/map/switchMap.do',
        type: 'POST',
        data: JSON.stringify({"map_seq" : $("#mapSelect").val()}),
        contentType: 'application/json',
        success: function(data) {
            if (data.result === "success") {
                waypoints = [];
                fn_refreshMap();
                fn_getNodes();

                toastr.success("지도가 전환되었습니다.", "지도 변경");
                $("#mapSelect, #poseBtn, #goalBtn, #nodeBtn").prop("disabled", false);
            } else {
                toastr.error(data.message, "지도 전환 실패");
                $("#mapSelect, #poseBtn, #goalBtn, #nodeBtn").prop("disabled", false);
            }
        },
        error: function() {
            toastr.error("서버 응답이 없습니다.", "지도 전환 실패");
            $("#mapSelect, #poseBtn, #goalBtn, #nodeBtn").prop("disabled", false);
        }
    });
}

function fn_addMap() {
    toastr.info("SLAM 모드로 전환 중입니다. 기존 노드가 종료될 수 있습니다.");
    $("#addMapBtn").hide();
    $("#saveMapBtn").show();
    

    // 화면 초기화
    if (poseTopic) poseTopic.unsubscribe();
    if (scanTopic) scanTopic.unsubscribe();
    if (laserContainer) laserContainer.removeAllChildren();
    if (nodeContainer) nodeContainer.removeAllChildren();
    if (gridClient) {
        gridClient.removeAllListeners('change');
        if (gridClient.currentGrid) mapContainer.removeChild(gridClient.currentGrid);
        gridClient = null;
    }
    mapContainer.removeAllChildren();
    viewer.scene.update();

    $.ajax({
        url: '/map/drawMap.do',
        type: 'POST',
        success: function(data) {
            if (data.result !== "success") {
                toastr.error(data.message);
                $("#addMapBtn").prop("disabled", false);
                return;
            }
            toastr.info("지도 위에 로봇이 나오면 조이스틱으로 로봇을 움직여 맵을 완성하세요.", "맵 생성 중", {timeOut: 0, closeButton: true});

            // 실시간 맵 구독
            setTimeout(function() {
                fn_startSlamMap();
            }, 5000);
        },
        error: function() {
            toastr.error("SLAM 전환 실패");
            $("#addMapBtn").show();
            $("#saveMapBtn").hide();
        }
    });
}

var slamPoseInterval = null;

function fn_startSlamMap() {
    // gridClient 초기화
    if (gridClient) {
        gridClient.removeAllListeners('change');
        gridClient = null;
    }
    fn_stopSlamMap();
    mapContainer.removeAllChildren();

    var mapReceived = false; // 첫 수신 여부 플래그
    var bitmap = null;       // bitmap 재사용

    // 동적으로 지도 크기를 조절하기 위해 이전 사이즈를 저장할 변수 생성
    var lastWidth = 0;
    var lastHeight = 0;

    var mapTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid',
        throttle_rate: 1000 // 1초에 한번 수신 (1Hz)
    });

    mapTopic.subscribe(function(message) {
        mapContainer.removeAllChildren();

        // OccupancyGrid를 직접 그리기
        var info = message.info;
        var width = info.width;
        var height = info.height;
        var resolution = info.resolution;
        var originX = info.origin.position.x;
        var originY = info.origin.position.y;

        if (width === 0 || height === 0) return;

        var canvas = document.createElement('canvas');
        canvas.width = width;
        canvas.height = height;
        var ctx = canvas.getContext('2d');
        var imageData = ctx.createImageData(width, height);

        for (var i = 0; i < message.data.length; i++) {
            var val = message.data[i];
            var r, g, b;
            if (val === -1) {       // unknown → 회색
                r = 128; g = 128; b = 128;
            } else if (val === 0) { // free → 흰색
                r = 255; g = 255; b = 255;
            } else {                // occupied → 검정
                r = 0; g = 0; b = 0;
            }
            // OccupancyGrid는 y축이 반전
            var row = height - 1 - Math.floor(i / width);
            var col = i % width;
            var idx = (row * width + col) * 4;
            imageData.data[idx]     = r;
            imageData.data[idx + 1] = g;
            imageData.data[idx + 2] = b;
            imageData.data[idx + 3] = 255;
        }
        ctx.putImageData(imageData, 0, 0);

        if (bitmap) {
            mapContainer.removeChild(bitmap);
        }

        bitmap = new createjs.Bitmap(canvas);
        bitmap.scaleX =  resolution;
        bitmap.scaleY =  resolution;
        bitmap.x =  originX;
        bitmap.y = -(originY + height * resolution);
        mapContainer.addChild(bitmap);


        // 실시간으로 받아온 맵 크기와 이전 맵 크기가 다르면 뷰 재조정
        if (width !== lastWidth || height !== lastHeight) {
            lastWidth  = width;
            lastHeight = height;
            var mapW = width  * resolution;
            var mapH = height * resolution;
            viewer.scaleToDimensions(mapW, mapH);
            viewer.shift(originX, originY);
        }

        if (!mapReceived) {
            mapReceived = true;
            viewer.scaleToDimensions(width * resolution, height * resolution);
            viewer.shift(originX, originY);
        }
        viewer.scene.update();
    });

    window.slamMapTopic = mapTopic;

    // 스캔 + 로봇 위치 추적 시작
    fn_subscriberTopic();
    fn_startSlamPoseTracking();
    robotContainer.visible = true;

    console.log("SLAM 맵 구독 시작");
}

function fn_stopSlamMap() {
    if (window.slamMapTopic) {
        window.slamMapTopic.unsubscribe();
        window.slamMapTopic = null;
    }
}

function fn_startSlamPoseTracking() {
    if (window.slamTfTopic) {
        window.slamTfTopic.unsubscribe();
        window.slamTfTopic = null;
    }

    var tfTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tf',
        messageType: 'tf2_msgs/TFMessage',
        throttle_rate: 100
    });

    // map→odom 변환값 저장
    var mapToOdom = null;
    var pendingUpdate = false;

    tfTopic.subscribe(function(message) {
        for (var i = 0; i < message.transforms.length; i++) {
            var t = message.transforms[i];

            // map → odom 저장
            if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
                mapToOdom = t.transform;
            }

            // odom → base_footprint + mapToOdom 합산
            if (t.header.frame_id === 'odom' && t.child_frame_id === 'base_footprint' && mapToOdom) {
                var ox = mapToOdom.translation.x;
                var oy = mapToOdom.translation.y;
                var qm = mapToOdom.rotation;
                var yawMap = Math.atan2(
                    2 * (qm.w * qm.z + qm.x * qm.y),
                    1 - 2 * (qm.y * qm.y + qm.z * qm.z)
                );

                var bx = t.transform.translation.x;
                var by = t.transform.translation.y;
                var qb = t.transform.rotation;
                var yawOdom = Math.atan2(
                    2 * (qb.w * qb.z + qb.x * qb.y),
                    1 - 2 * (qb.y * qb.y + qb.z * qb.z)
                );

                // map 좌표계에서의 최종 위치
                var finalX = ox + bx * Math.cos(yawMap) - by * Math.sin(yawMap);
                var finalY = oy + bx * Math.sin(yawMap) + by * Math.cos(yawMap);
                var finalYaw = yawMap + yawOdom;

                robotContainer.x = finalX;
                robotContainer.y = -finalY;
                robotContainer.rotation = -finalYaw * (180 / Math.PI);
                robotContainer.visible = true;

                if (!pendingUpdate) {
                    pendingUpdate = true;
                    requestAnimationFrame(function() {
                        viewer.scene.update();
                        pendingUpdate = false;
                    });
                }
            }
        }
    });

    var tfStaticTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tf_static',
        messageType: 'tf2_msgs/TFMessage'
    });

    tfStaticTopic.subscribe(function(message) {
        for (var i = 0; i < message.transforms.length; i++) {
            var t = message.transforms[i];
            if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
                mapToOdom = t.transform;
            }
        }
    });

    window.slamTfTopic = tfTopic;
    window.slamTfStaticTopic = tfStaticTopic;
}

function fn_stopSlamPoseTracking() {
    if (window.slamTfTopic) {
        window.slamTfTopic.unsubscribe();
        window.slamTfTopic = null;
    }
    if (window.slamTfStaticTopic) {
        window.slamTfStaticTopic.unsubscribe();
        window.slamTfStaticTopic = null;
    }
}

function fn_saveMap() {
    var mapName = prompt("저장할 맵 이름을 입력하세요:", "new_map_" + Date.now());
    if (!mapName) return;
    toastr.info("맵 저장 중...", "저장");

    // SLAM 관련 구독 중지
    fn_stopSlamMap();
    fn_stopSlamPoseTracking();
    robotContainer.visible = false;

    $.ajax({
        url: '/map/saveMap.do',
        type: 'POST',
        data: JSON.stringify({"map_name": mapName}),
        contentType: 'application/json',
        success: function(data) {
            toastr.clear(); // 무한 알림 삭제
            if (data.result === "success") {
                toastr.success(data.message, "저장 완료");

                // selectbox에 새 맵 추가 후 선택
                var newOption = '<option value="' + data.map.map_seq + '" selected>' + data.map.map_name + '</option>';
                isInitializing = true; // selectbox change 이벤트 방지
                $("#mapSelect option").prop("selected", false);
                $("#mapSelect").append(newOption);
                isInitializing = false;

                $("#addMapBtn").show();
                $("#saveMapBtn").hide();

                setTimeout(function() {
                    fn_refreshMap();
                    fn_getNodes();
                    robotContainer.visible = true;
                }, 10000);

                

            } else {
                toastr.error(data.message, "저장 실패");
                $("#addMapBtn").show();
                $("#saveMapBtn").hide();
            }
        },
        error: function() {
            toastr.error("맵 저장 실패");
            $("#addMapBtn").show();
            $("#saveMapBtn").hide();
        }
    });
}

function fn_deleteMap() {
    var deleteMapSeq = $("#mapSelect").val();
    if (deleteMapSeq === 1) {
        toastr.error("기본 맵은 삭제가 불가능합니다.", "삭제 실패");
        return;
    }
    $.ajax({
        url: '/map/deleteMap.do',
        type: 'POST',
        data: JSON.stringify({"map_seq" : deleteMapSeq}),
        contentType: 'application/json',
        success: function(data) {
            if (data.result === "success") {
                toastr.success(data.message, "삭제 완료");
                fn_getMaps();
            } else {
                toastr.error(data.message, "삭제 실패");
            }
        },
        error: function() {
            toastr.error("맵 삭제 실패");
        }
    });
}

function fn_insertAlert(alert) {
    $.ajax({
        url : "/alert/insertAlert.do",
        type : "POST",
        contentType: 'application/json',
        dataType: 'json', 
        data: JSON.stringify({
            "alert_type": alert.alert_type,
            "alert_msg" : alert.alert_msg
        }),
        success: function(data) {
            fn_updateAlert(data.alerts);
        }
    });
}

function fn_updateAlert(alerts) {
    var container = $("#alerts");
    $("#alertCount").text(alerts.length);
    container.html("");

    $.each(alerts, function(idx, alert) {
        var isUnread = alert.check_yn === 'N';
        var boldClass = isUnread ? 'font-weight-bold' : '';

        var itemHtml = `
            <a class="dropdown-item d-flex align-items-center" href="#" onclick="fn_checkAlert(${alert.alert_seq}, this)">
                <div class="mr-3">
                    <div class="icon-circle bg-${isUnread ? 'warning' : 'secondary'}">
                        <i class="fas fa-bell text-white"></i>
                    </div>
                </div>
                <div>
                    <div class="small text-gray-500">${alert.created_at}</div>
                    <span class="${boldClass}">${alert.alert_msg}</span>
                </div>
            </a>
        `;
        container.append(itemHtml);
    });
}

function fn_checkAlert(seq, element) {
    $.ajax({
        url: "/alert/checkAlert.do",
        type: "POST",
        contentType: 'application/json',
        data: JSON.stringify({ "alert_seq": seq }),
        success: function(data) {
            if (data.result === "success") {
                fn_updateAlert(data.alerts)
            }
        }
    });
}

function fn_startDrive() {
    palletId = parseInt($("#selectPallet").val());
    $.ajax({
        url: "/drive/startDrive.do",
        type: "POST",
        contentType: 'application/json',
        data: JSON.stringify({ "palletId": palletId }),
        success: function(data) {
            if (data.result === "success") {
                toastr.success(data.message);
            }
        },
        error: function() {
            toastr.error("주행 실패");
        }
    });
}

// 중단 및 복귀 기능
function fn_abortAndReturn() {
    $.ajax({
        url: "/drive/abortReturn.do",
        type: "POST",
        contentType: 'application/json',
        data: JSON.stringify({}),
        success: function(data) {
            if (data.result === "success") {
                toastr.warning(data.message);
            }
        },
        error: function() {
            toastr.error("중단 및 복귀 요청 실패");
        }
    });
}

// 비상정지 기능
function fn_emergencyStop() {
    $.ajax({
        url: "/drive/emergencyStop.do",
        type: "POST",
        contentType: 'application/json',
        data: JSON.stringify({}),
        success: function(data) {
            if (data.result === "success") {
                toastr.error(data.message);
            }
        },
        error: function() {
            toastr.error("비상 정지 요청 실패");
        }
    });
}
