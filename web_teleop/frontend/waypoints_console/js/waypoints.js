document.addEventListener("DOMContentLoaded", function(event) {
    var viewer = new ROS3D.Viewer({
        divID : 'markers',
        width : document.body.clientWidth * .4,
        height : document.body.clientWidth * .4,
        antialias : true,
        background : "#f0f0ee"
    });
    viewer.addObject(new ROS3D.Grid());
    
    // TF Server
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/odom'
    });
    
    var imClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/basic_controls',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
    });

    // Add Grid
    var gridClient = new ROS3D.OccupancyGridClient({
           ros : ros,
           rootObject : viewer.scene
    });

    // Robot Model
    var urdfClient = new ROS3D.UrdfClient({
       ros : ros,
       tfClient : tfClient,
       path : '/static/',
       rootObject : viewer.scene,
     });

    /*var odometryClient = new ROS3D.Odometry({
        ros: ros,
        topic: '/mobile_base_controller/odom',
        tfClient: tfClient,
        rootObject: viewer
    });*/
    
 });