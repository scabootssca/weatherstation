// This was borrowed from darksky.net all credit goes to them //
var Slider = function(container, params) {

  var $container = $(container),
      $handle = $container.find('.handle'),
      dragging = false,
      animating_to = false,
      container_x,
      container_width,
      mouse_x, start_mouse_x,
      did_move = false,
      handle_x = 0,
      val = 0,
      speed,
      last_time

  var _change = params.change,
      _start = params.start,
      _stop = params.stop,
      _stop_animating_to = params.stop_animating_to,
      _move_on_down = params.move_on_down == null ? true : params.move_on_down

  var max_speed = (params.max_speed != null) ? params.max_speed : 0.35,
      speed_multiple = (params.speed_multiple != null) ? params.speed_multiple : 0.01,
      stop_decay = (params.stop_decay != null) ? params.stop_decay : 0.004,
      snap_distance = (params.snap_distance != null) ? params.snap_distance : 50


  // requestAnimationFrame shim
  // http://paulirish.com/2011/requestanimationframe-for-smart-animating/
  var requestAnimFrame = (function() {
    return  window.requestAnimationFrame       ||
            window.webkitRequestAnimationFrame ||
            window.mozRequestAnimationFrame    ||
            window.oRequestAnimationFrame      ||
            window.msRequestAnimationFrame     ||
            function(/* function */ callback, /* DOMElement */ element) {
              window.setTimeout(callback, 11)
            }
  })()

  var sgn = function(val) {
    return val < 0 ? -1 : 1
  }


  var animate = function() {

    var time = new Date().getTime(),
        dt = (time - last_time) || 0

    if(dragging || animating_to) {
      speed = speed_multiple * (mouse_x - handle_x)

      if(Math.abs(speed) > max_speed)
        speed = sgn(speed) * max_speed

    } else {
      speed *= Math.exp(-dt * stop_decay)
    }

    if(Math.abs(speed) < 0.005) speed = 0
    handle_x = Math.min(container_width, Math.max(0, handle_x + speed * dt))
    val = handle_x / container_width

    $handle.css('left', handle_x)

    last_time = time

    if(_change) _change(val)

    if(dragging || speed != 0) {
      requestAnimFrame(animate)

    } else if(animating_to && _stop_animating_to) {
      animating_to = false
      _stop_animating_to()
    } else if(_stop) {
      _stop()
    }
  }


  var on_down = function(e) {
    e.stopPropagation()
    e.preventDefault()

    container_x = $container.offset().left
    container_width = $container.width()

    dragging = true
    did_move = false

    var touches = e.touches || e.originalEvent.touches
    start_mouse_x = mouse_x = (touches ? touches[0].pageX : e.clientX) - container_x

    last_time = new Date().getTime()

    $touchmove_elem.unbind('mouseup', on_up)
    $touchmove_elem.unbind('touchend', on_up)
    $touchmove_elem.bind('mouseup', on_up)
    $touchmove_elem.bind('touchend', on_up)

    $touchmove_elem.unbind('mousemove', on_move)
    $touchmove_elem.unbind('touchmove', on_move)
    $touchmove_elem.bind('mousemove', on_move)
    $touchmove_elem.bind('touchmove', on_move)

    // If the user clicks far enough away from the handle, jump to mouse position
    if(_move_on_down) {
      if(Math.abs(handle_x - mouse_x) > snap_distance) {
        handle_x = mouse_x
        val = handle_x / container_width
        $handle.css('left', handle_x)

        if(_start) _start()
        if(_change) _change(val)

      } else {
        if(_start) _start()
      }

      animate()
    }
  }


  var on_up = function() {
    dragging = false

    $touchmove_elem.unbind('mouseup', on_up)
    $touchmove_elem.unbind('mousemove', on_move)
    $touchmove_elem.unbind('touchend', on_up)
    $touchmove_elem.unbind('touchmove', on_move)

    if(!did_move)
      $container.trigger('tap')
  }


  var on_move = function(e) {
    e.preventDefault()

    var touches = e.touches || e.originalEvent.touches
    mouse_x = (touches ? touches[0].pageX : e.clientX) - container_x

    if(!did_move && Math.abs(mouse_x - start_mouse_x) > 10) {
      did_move = true
      $container.trigger('start_move')

      if(!_move_on_down) {
        if(_start) _start()
        animate()
      }
    }
  }


  var set_val = function(new_val) {
    val = new_val
    handle_x = val * $container.width()
    $handle.css('left', handle_x)
  }


  var animate_to = function(new_val) {
    if(dragging) return
    if(container_width == null) container_width = $container.width()
    mouse_x = new_val * container_width
    animating_to = true
    animate()
  }


  $container.bind('mousedown', on_down)
  $container.bind('touchstart', on_down)

  $touchmove_elem = $('body') // FIXME: ENV.is_android ? $container : $('body')

  $container.set_val = set_val
  $container.animate_to = animate_to

  return $container
}
