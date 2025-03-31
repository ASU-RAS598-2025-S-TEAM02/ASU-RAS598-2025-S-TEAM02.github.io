document.addEventListener("DOMContentLoaded", function () {
    function loadFooter() {
        fetch("modules/footer/footer.html")
            .then(response => response.text())
            .then(data => {
                document.getElementById("footer-placeholder").innerHTML = data;
            })
            .catch(error => console.error("Error loading footer:", error));
    }
    loadFooter();

    // setInterval(loadFooter, 5000);
});
