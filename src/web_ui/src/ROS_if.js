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
        stdout_ptr.value = "";
        return;
    }

    if (splitted_stdin.length < 3) {
        stdout_ptr.value = "Invalid command";
        stdout_ptr.className = "textarea is-warning";
        return;
    }

    if (splitted_stdin[0] != "ros2") {
        stdout_ptr.value = "Invalid command";
        stdout_ptr.className = "textarea is-warning";
        return;
    }

    if (splitted_stdin[1] != "topic" && splitted_stdin[1] != "service" && splitted_stdin[1] != "node") {
        stdout_ptr.value = "Invalid command";
        stdout_ptr.className = "textarea is-warning";
        return;
    }

    if (stdin_msg == "ros2 topic list" || stdin_msg == "ros2 topic list ") {
        get_all_topics(stdout_ptr);
        return;
    }

    // Disini sudah 4 kata
    //--------------------
    if (splitted_stdin.length < 4) {
        stdout_ptr.value = "Invalid command";
        stdout_ptr.className = "textarea is-warning";
        return;
    }

    if (splitted_stdin[1] == "topic") {
        parse_topic_cmd(listener_id, stdout_ptr, splitted_stdin[2], splitted_stdin[3]);
    }

}

function parse_topic_cmd(listener_id, stdout_ptr, command, topic_name) {
    if (command == "echo") {
        echo_topic(listener_id, stdout_ptr, topic_name);
    }
    else if (command == "hz") {
        hz_topic(listener_id, stdout_ptr, topic_name);
    }
    else {
        stdout_ptr.value = "Invalid command";
        stdout_ptr.className = "textarea is-warning";
        return;
    }
}

function hz_topic(listener_id, stdout_ptr, topic_name) {
    var topicsService = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    topicsService.callService(request, function (result) {
        let topics = result.topics;
        let types = result.types;

        let topic_ditemukan = false;
        let fixed_topic_name = topic_name;
        let fixed_topic_type = "";
        let prev_time = new Date().getTime();

        for (let i = 0; i < topics.length; i++) {
            if (topics[i] == topic_name) {
                topic_ditemukan = true;
                fixed_topic_type = types[i];
                break;
            }
        }

        if (!topic_ditemukan) {
            stdout_ptr.value = "Topic not found";
            stdout_ptr.className = "textarea is-warning";
            return;
        }

        stdout_ptr.className = "textarea is-primary";

        if (listener_id == 1) {
            if (listener1 != null) {
                listener1.unsubscribe();
                listener1 = null;
            }

            listener1 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener1.subscribe(function (message) {
                let curr_time = new Date().getTime();
                let hz = 1000 / (curr_time - prev_time);
                prev_time = curr_time

                stdout_ptr.value += `${hz.toFixed(3)} Hz\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 2) {
            if (listener2 != null) {
                listener2.unsubscribe();
                listener2 = null;
            }

            listener2 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener2.subscribe(function (message) {
                let curr_time = new Date().getTime();
                let hz = 1000 / (curr_time - prev_time);
                prev_time = curr_time

                stdout_ptr.value += `${hz.toFixed(3)} Hz\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 3) {
            if (listener3 != null) {
                listener3.unsubscribe();
                listener3 = null;
            }

            listener3 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener3.subscribe(function (message) {
                let curr_time = new Date().getTime();
                let hz = 1000 / (curr_time - prev_time);
                prev_time = curr_time

                stdout_ptr.value += `${hz.toFixed(3)} Hz\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 4) {
            if (listener4 != null) {
                listener4.unsubscribe();
                listener4 = null;
            }

            listener4 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener4.subscribe(function (message) {
                let curr_time = new Date().getTime();
                let hz = 1000 / (curr_time - prev_time);
                prev_time = curr_time

                stdout_ptr.value += `${hz.toFixed(3)} Hz\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }

    });
}

function echo_topic(listener_id, stdout_ptr, topic_name) {
    var topicsService = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    topicsService.callService(request, function (result) {
        let topics = result.topics;
        let types = result.types;

        let topic_ditemukan = false;
        let fixed_topic_name = topic_name;
        let fixed_topic_type = "";

        for (let i = 0; i < topics.length; i++) {
            if (topics[i] == topic_name) {
                topic_ditemukan = true;
                fixed_topic_type = types[i];
                break;
            }
        }

        if (!topic_ditemukan) {
            stdout_ptr.value = "Topic not found";
            stdout_ptr.className = "textarea is-warning";
            return;
        }

        stdout_ptr.className = "textarea is-primary";

        if (listener_id == 1) {
            if (listener1 != null) {
                listener1.unsubscribe();
                listener1 = null;
            }

            listener1 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener1.subscribe(function (message) {
                stdout_ptr.value += `${handle_message_print(message)}\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 2) {
            if (listener2 != null) {
                listener2.unsubscribe();
                listener2 = null;
            }

            listener2 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener2.subscribe(function (message) {
                stdout_ptr.value += `${handle_message_print(message)}\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 3) {
            if (listener3 != null) {
                listener3.unsubscribe();
                listener3 = null;
            }

            listener3 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener3.subscribe(function (message) {
                stdout_ptr.value += `${handle_message_print(message)}\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }
        else if (listener_id == 4) {
            if (listener4 != null) {
                listener4.unsubscribe();
                listener4 = null;
            }

            listener4 = new ROSLIB.Topic({
                ros: ros,
                name: fixed_topic_name,
                messageType: fixed_topic_type,
            });

            listener4.subscribe(function (message) {
                stdout_ptr.value += `${handle_message_print(message)}\n`;
                stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
            });
        }

    });
}

function handle_message_print(message) {
    if (typeof message === "object") {
        return JSON.stringify(message, null, 2); // Pretty print JSON objects
    } else {
        return String(message); // Convert other types to string
    }
}

function get_all_topics(stdout_ptr) {
    var topicsService = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    stdout_ptr.className = "textarea is-primary";

    topicsService.callService(request, function (result) {
        let topics = result.topics;
        let types = result.types;

        for (let i = 0; i < topics.length; i++) {
            stdout_ptr.value += `${topics[i]} : ${types[i]}\n`;
            stdout_ptr.scrollTop = stdout_ptr.scrollHeight;
        }
    });
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
        if (listener1 != null) {
            listener1.unsubscribe();
            listener1 = null;
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
        if (listener2 != null) {
            listener2.unsubscribe();
            listener2 = null;
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
        if (listener3 != null) {
            listener3.unsubscribe();
            listener3 = null;
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
        if (listener4 != null) {
            listener4.unsubscribe();
            listener4 = null;
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
        if (listener1 != null) {
            listener1.unsubscribe();
            listener1 = null;
        }
    }
});
stdout2.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (listener2 != null) {
            listener2.unsubscribe();
            listener2 = null;
        }
    }
});
stdout3.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (listener3 != null) {
            listener3.unsubscribe();
            listener3 = null;
        }
    }
});
stdout4.addEventListener('keydown', function (event) {
    if (event.key == "Escape") {
        if (listener4 != null) {
            listener4.unsubscribe();
            listener4 = null;
        }
    }
});

//==============================================================================

setInterval(() => {
}, 50);
