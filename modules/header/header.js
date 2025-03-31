document.addEventListener("DOMContentLoaded", function () {
    function loadHeader() {
        fetch("modules/header/header.html")
            .then(response => response.text())
            .then(data => {
                document.getElementById("header-placeholder").innerHTML = data;
            })
            .catch(error => console.error("Error loading header:", error));
    }
    // Initialize the header on page load
    loadHeader();

    // setInterval(loadHeader, 5000);
});
