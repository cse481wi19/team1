document.addEventListener("DOMContentLoaded", function(event) {
    var viewer = new ROS3D.Viewer({
        divID : 'markers',
        width : document.body.clientWidth * .4,
        height : document.body.clientWidth * .4,
        antialias : true,
        background : "#f0f0ee"
    });
    viewer.addObject(new ROS3D.Grid());
    
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/rotating_frame'
    });
    
    var imClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/basic_controls',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
    });

    var gridClient = new ROS3D.OccupancyGridClient({
           ros : ros,
           rootObject : viewer.scene
    });

    var urdfClient = new ROS3D.UrdfClient({
       ros : ros,
       tfClient : tfClient,
       path : '/static/gizmo_description',
       rootObject : viewer.scene,
       loader : ROS3D.COLLADA_LOADER
     });
 });