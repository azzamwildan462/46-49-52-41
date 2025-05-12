// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});

//==============================================================================

const stdin1 = document.getElementById('stdin1');
const stdin2 = document.getElementById('stdin2');
const stdin3 = document.getElementById('stdin3');
const stdin4 = document.getElementById('stdin4');
const stdout1 = document.getElementById('stdout1');
const stdout2 = document.getElementById('stdout2');
const stdout3 = document.getElementById('stdout3');
const stdout4 = document.getElementById('stdout4');

var stdin1_val = "";
var stdin2_val = "";
var stdin3_val = "";
var stdin4_val = "";
var stdout1_val = "";
var stdout2_val = "";
var stdout3_val = "";
var stdout4_val = "";

let listener1 = null;
let listener2 = null;
let listener3 = null;
let listener4 = null;

let stdin1_history = [];
let stdin2_history = [];
let stdin3_history = [];
let stdin4_history = [];

let stdin1_history_index = 0;
let stdin2_history_index = 0;
let stdin3_history_index = 0;
let stdin4_history_index = 0;

//==============================================================================

async function parse_stdin(listener_id, stdout_ptr, stdin_msg) {
    const splitted_stdin = stdin_msg.split(" ");

    if (stdin_msg == "clear" || stdin_msg == "clear ") {
        stdout_ptr.src = "";
        stdout_ptr.alt = "MJPEG Stream";
        return;
    }

    if (splitted_stdin.length < 1) {
        stdout_ptr.src = "";
        stdout_ptr.alt = "Insert Topic Name";
        return;
    }

    if (splitted_stdin.length < 2) {
        process_mjpeg_req(listener_id, stdout_ptr, splitted_stdin[0], -1);
        return;
    }

    process_mjpeg_req(listener_id, stdout_ptr, splitted_stdin[0], splitted_stdin[1]);

    return;
}

function process_mjpeg_req(listener_id, stdout_ptr, topic, quality) {
    if (quality == -1) {
        stdout_ptr.src = "http://" + window.location.hostname + ":8080/stream?topic=" + topic;
        return;
    }

    if (quality < 1)
        quality = 1;
    else if (quality > 100)
        quality = 100;

    stdout_ptr.src = "http://" + window.location.hostname + ":8080/stream?topic=" + topic + "&quality=" + quality;
}

//==============================================================================

stdin1.addEventListener('keydown', async function (event) {
    if (event.key == "Enter") {
        stdin1_val = stdin1.value;
        stdin1_history.push(stdin1_val);
        stdin1_history_index = stdin1_history.length;
        parse_stdin(1, stdout1, stdin1_val);
        stdin1.value = "";
    }
    else if (event.key == "Escape") {
        if (stdout1.src != "") {
            stdout1.src = "";
            stdout1.alt = "MJPEG Stream";
        }
    }
    else if (event.key == "ArrowUp") {
        if (stdin1_history.length > 0) {
            if (stdin1_history_index > 0) {
                stdin1_history_index--;
                stdin1.value = stdin1_history[stdin1_history_index];
            }
        }
    }
    else if (event.key == "ArrowDown") {
        if (stdin1_history.length > 0) {
            if (stdin1_history_index < stdin1_history.length - 1) {
                stdin1_history_index++;
                stdin1.value = stdin1_history[stdin1_history_index];
            }
        }
    }
});
stdin2.addEventListener('keydown', function (event) {
    if (event.key == "Enter") {
        stdin2_val = stdin2.value;
        stdin2_history.push(stdin2_val);
        stdin2_history_index = stdin2_history.length;
        parse_stdin(2, stdout2, stdin2_val);
        stdin2.value = "";
    }
    else if (event.key == "Escape") {
        if (stdout2.src != "") {
            stdout2.src = "";
            stdout2.alt = "MJPEG Stream";
        }
    }
    else if (event.key == "ArrowUp") {
        if (stdin2_history.length > 0) {
            if (stdin2_history_index > 0) {
                stdin2_history_index--;
                stdin2.value = stdin2_history[stdin2_history_index];
            }
        }
    }
    else if (event.key == "ArrowDown") {
        if (stdin2_history.length > 0) {
            if (stdin2_history_index < stdin2_history.length - 1) {
                stdin2_history_index++;
                stdin2.value = stdin2_history[stdin2_history_index];
            }
        }
    }
});
stdin3.addEventListener('keydown', function (event) {
    if (event.key == "Enter") {
        stdin3_val = stdin3.value;
        stdin3_history.push(stdin3_val);
        stdin3_history_index = stdin3_history.length;
        parse_stdin(3, stdout3, stdin3_val);
        stdin3.value = "";
    }
    else if (event.key == "Escape") {
        if (stdout3.src != "") {
            stdout3.src = "";
            stdout3.alt = "MJPEG Stream";
        }
    }
    else if (event.key == "ArrowUp") {
        if (stdin3_history.length > 0) {
            if (stdin3_history_index > 0) {
                stdin3_history_index--;
                stdin3.value = stdin3_history[stdin3_history_index];
            }
        }
    }
    else if (event.key == "ArrowDown") {
        if (stdin3_history.length > 0) {
            if (stdin3_history_index < stdin3_history.length - 1) {
                stdin3_history_index++;
                stdin3.value = stdin3_history[stdin3_history_index];
            }
        }
    }
});
stdin4.addEventListener('keydown', function (event) {
    if (event.key == "Enter") {
        stdin4_val = stdin4.value;
        stdin4_history.push(stdin4_val);
        stdin4_history_index = stdin4_history.length;
        parse_stdin(4, stdout4, stdin4_val);
        stdin4.value = "";
    }
    else if (event.key == "Escape") {
        if (stdout4.src != "") {
            stdout4.src = "";
            stdout4.alt = "MJPEG Stream";
        }
    }
    else if (event.key == "ArrowUp") {
        if (stdin4_history.length > 0) {
            if (stdin4_history_index > 0) {
                stdin4_history_index--;
                stdin4.value = stdin4_history[stdin4_history_index];
            }
        }
    }
    else if (event.key == "ArrowDown") {
        if (stdin4_history.length > 0) {
            if (stdin4_history_index < stdin4_history.length - 1) {
                stdin4_history_index++;
                stdin4.value = stdin4_history[stdin4_history_index];
            }
        }
    }
});
stdout1.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (stdout1.src != "") {
            stdout1.src = "";
            stdout1.alt = "MJPEG Stream";
        }
    }
});
stdout2.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (stdout2.src != "") {
            stdout2.src = "";
            stdout2.alt = "MJPEG Stream";
        }
    }
});
stdout3.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (stdout3.src != "") {
            stdout3.src = "";
            stdout3.alt = "MJPEG Stream";
        }
    }
});
stdout4.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (stdout4.src != "") {
            stdout4.src = "";
            stdout4.alt = "MJPEG Stream";
        }
    }
});

//==============================================================================

setInterval(() => {
}, 50);
