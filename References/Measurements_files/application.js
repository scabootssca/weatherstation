$(document).ready(function() {

	// Site search box
	var $search = $('#sitesearch');
	original_val = $search.val();

	$search.focus(function(){
		if($(this).val()===original_val){
			$(this).val('');
			$(this).css({color:'#444444'});
		}
	}).blur(function(){
		if($(this).val()===''){
			$(this).css({color:'#aaaaaa'});
			$(this).val(original_val);
		}
	});

	$('.submitbutton').click(function() {
		if ($('#sitesearch').val()===original_val) {
			alert('Please enter a search term in the\nsearch box before submitting.');
			return false;
		}
	});

	// Open external links in a new window: use rel="external"
	$('a[rel="external"]').click(function() {window.open($(this).attr('href')); return false;});

	// modify external links to display external link icon and display the exit message
	// on links that link to sites that aren't .gov domains
	$('a').each(function() {
		var href = $(this).attr('href');
		if (typeof href != 'undefined') {
			if (href.slice(0, 'mailto:'.length) != 'mailto:'
				&& (href.slice(0, 'http://'.length) == 'http://'
				|| href.slice(0, 'https://'.length) == 'https://'
				|| href.slice(0, 'ftp://'.length) == 'ftp://'
				|| href.slice(0, '//'.length) == '//'
				|| href.slice(0, 'www'.length) == 'www'
				|| (href.split('/') > 1 && href.split('/')[0].split('.').length > 1))) {

				var domain = href.replace('http://','').replace('https://','').replace('ftp://','').replace('//','').split('/')[0];
				var domains = domain.split('.');
				var tld = domains[domains.length - 1];
				if (tld != 'gov') {
					$(this).after('<span class="ext"></span>');
					$(this).click(function(e) {
						return confirm('You are exiting an NCEI website.\n\nThank you for visiting our site. We have provided a link because it has information that may interest you. NCEI does not endorse the views expressed, the information presented, or any commercial products that may be advertised or available on that site.');
					});
				}
			}
		}
	});

	// Universal Expandable Content Code
	$('.expandableContent').hide();

	// Switch the "Open" and "Close" state per click then slide up/down depending on open/close state
	$('.expandableHeader').click(function() {
		$(this).toggleClass('expanded').next().slideToggle();
		return false;
	});

});