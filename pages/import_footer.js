function loadFooter() {
    const placeholder = document.getElementById('footer-placeholder');
    if (placeholder) {
        fetch('footer.html')
            .then(response => {
                if (!response.ok) throw new Error('Nepodařilo se načíst footer.');
                return response.text();
            })
            .then(data => {
                placeholder.innerHTML = data;
            })
            .catch(error => console.error('Chyba:', error));
    }
}

window.addEventListener('DOMContentLoaded', loadFooter);