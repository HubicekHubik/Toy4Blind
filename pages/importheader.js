function loadHeader() {
    const placeholder = document.getElementById('header-placeholder');
    if (placeholder) {
        fetch('header.html')
            .then(response => {
                if (!response.ok) throw new Error('Nepodařilo se načíst header.');
                return response.text();
            })
            .then(data => {
                placeholder.innerHTML = data;
            })
            .catch(error => console.error('Chyba:', error));
    }
}

window.addEventListener('DOMContentLoaded', loadHeader);