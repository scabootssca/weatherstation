function doNothing(percent) {
}

var $slider = $("#slider"),
	top = 160,
	height = 1600,
	left = "5%",
	width = "90%"

$slider.css({top: top, height: height, left: left, width: width})

//$slider.show()

var slider = new Slider($slider, {
	change: doNothing,
	max_speed: 999,
	snap_distance: 30
})
