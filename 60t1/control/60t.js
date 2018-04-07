function move(l,r) {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
        if (this.readyState == 4) {
            if (this.status != 200) {
                window.alert("Command Failed");
            }
        }
    };
    xhr.open("GET", "/cgi-bin/60t?cmd=set&l="+l+"&r="+r);
    xhr.send();
}
