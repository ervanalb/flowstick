// GENERATED CODE - DO NOT MODIFY BY HAND
// coverage:ignore-file
// ignore_for_file: type=lint
// ignore_for_file: unused_element, deprecated_member_use, deprecated_member_use_from_same_package, use_function_type_syntax_for_parameters, unnecessary_const, avoid_init_to_null, invalid_override_different_default_values_named, prefer_expression_function_bodies, annotate_overrides, invalid_annotation_target, unnecessary_question_mark

part of 'state.dart';

// **************************************************************************
// FreezedGenerator
// **************************************************************************

// dart format off
T _$identity<T>(T value) => value;
/// @nodoc
mixin _$MyState {





@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is MyState);
}


@override
int get hashCode => runtimeType.hashCode;

@override
String toString() {
  return 'MyState()';
}


}

/// @nodoc
class $MyStateCopyWith<$Res>  {
$MyStateCopyWith(MyState _, $Res Function(MyState) __);
}


/// Adds pattern-matching-related methods to [MyState].
extension MyStatePatterns on MyState {
/// A variant of `map` that fallback to returning `orElse`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeMap<TResult extends Object?>({TResult Function( Initializing value)?  initializing,TResult Function( Scanning value)?  scanning,TResult Function( Connecting value)?  connecting,TResult Function( Connected value)?  connected,TResult Function( Error value)?  error,required TResult orElse(),}){
final _that = this;
switch (_that) {
case Initializing() when initializing != null:
return initializing(_that);case Scanning() when scanning != null:
return scanning(_that);case Connecting() when connecting != null:
return connecting(_that);case Connected() when connected != null:
return connected(_that);case Error() when error != null:
return error(_that);case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// Callbacks receives the raw object, upcasted.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case final Subclass2 value:
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult map<TResult extends Object?>({required TResult Function( Initializing value)  initializing,required TResult Function( Scanning value)  scanning,required TResult Function( Connecting value)  connecting,required TResult Function( Connected value)  connected,required TResult Function( Error value)  error,}){
final _that = this;
switch (_that) {
case Initializing():
return initializing(_that);case Scanning():
return scanning(_that);case Connecting():
return connecting(_that);case Connected():
return connected(_that);case Error():
return error(_that);}
}
/// A variant of `map` that fallback to returning `null`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? mapOrNull<TResult extends Object?>({TResult? Function( Initializing value)?  initializing,TResult? Function( Scanning value)?  scanning,TResult? Function( Connecting value)?  connecting,TResult? Function( Connected value)?  connected,TResult? Function( Error value)?  error,}){
final _that = this;
switch (_that) {
case Initializing() when initializing != null:
return initializing(_that);case Scanning() when scanning != null:
return scanning(_that);case Connecting() when connecting != null:
return connecting(_that);case Connected() when connected != null:
return connected(_that);case Error() when error != null:
return error(_that);case _:
  return null;

}
}
/// A variant of `when` that fallback to an `orElse` callback.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeWhen<TResult extends Object?>({TResult Function()?  initializing,TResult Function( List<ScanResult> scanResults)?  scanning,TResult Function()?  connecting,TResult Function( ConnectedState connectedState)?  connected,TResult Function( String errorMessage)?  error,required TResult orElse(),}) {final _that = this;
switch (_that) {
case Initializing() when initializing != null:
return initializing();case Scanning() when scanning != null:
return scanning(_that.scanResults);case Connecting() when connecting != null:
return connecting();case Connected() when connected != null:
return connected(_that.connectedState);case Error() when error != null:
return error(_that.errorMessage);case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// As opposed to `map`, this offers destructuring.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case Subclass2(:final field2):
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult when<TResult extends Object?>({required TResult Function()  initializing,required TResult Function( List<ScanResult> scanResults)  scanning,required TResult Function()  connecting,required TResult Function( ConnectedState connectedState)  connected,required TResult Function( String errorMessage)  error,}) {final _that = this;
switch (_that) {
case Initializing():
return initializing();case Scanning():
return scanning(_that.scanResults);case Connecting():
return connecting();case Connected():
return connected(_that.connectedState);case Error():
return error(_that.errorMessage);}
}
/// A variant of `when` that fallback to returning `null`
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? whenOrNull<TResult extends Object?>({TResult? Function()?  initializing,TResult? Function( List<ScanResult> scanResults)?  scanning,TResult? Function()?  connecting,TResult? Function( ConnectedState connectedState)?  connected,TResult? Function( String errorMessage)?  error,}) {final _that = this;
switch (_that) {
case Initializing() when initializing != null:
return initializing();case Scanning() when scanning != null:
return scanning(_that.scanResults);case Connecting() when connecting != null:
return connecting();case Connected() when connected != null:
return connected(_that.connectedState);case Error() when error != null:
return error(_that.errorMessage);case _:
  return null;

}
}

}

/// @nodoc


class Initializing implements MyState {
  const Initializing();
  






@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Initializing);
}


@override
int get hashCode => runtimeType.hashCode;

@override
String toString() {
  return 'MyState.initializing()';
}


}




/// @nodoc


class Scanning implements MyState {
  const Scanning(final  List<ScanResult> scanResults): _scanResults = scanResults;
  

 final  List<ScanResult> _scanResults;
 List<ScanResult> get scanResults {
  if (_scanResults is EqualUnmodifiableListView) return _scanResults;
  // ignore: implicit_dynamic_type
  return EqualUnmodifiableListView(_scanResults);
}


/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@JsonKey(includeFromJson: false, includeToJson: false)
@pragma('vm:prefer-inline')
$ScanningCopyWith<Scanning> get copyWith => _$ScanningCopyWithImpl<Scanning>(this, _$identity);



@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Scanning&&const DeepCollectionEquality().equals(other._scanResults, _scanResults));
}


@override
int get hashCode => Object.hash(runtimeType,const DeepCollectionEquality().hash(_scanResults));

@override
String toString() {
  return 'MyState.scanning(scanResults: $scanResults)';
}


}

/// @nodoc
abstract mixin class $ScanningCopyWith<$Res> implements $MyStateCopyWith<$Res> {
  factory $ScanningCopyWith(Scanning value, $Res Function(Scanning) _then) = _$ScanningCopyWithImpl;
@useResult
$Res call({
 List<ScanResult> scanResults
});




}
/// @nodoc
class _$ScanningCopyWithImpl<$Res>
    implements $ScanningCopyWith<$Res> {
  _$ScanningCopyWithImpl(this._self, this._then);

  final Scanning _self;
  final $Res Function(Scanning) _then;

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@pragma('vm:prefer-inline') $Res call({Object? scanResults = null,}) {
  return _then(Scanning(
null == scanResults ? _self._scanResults : scanResults // ignore: cast_nullable_to_non_nullable
as List<ScanResult>,
  ));
}


}

/// @nodoc


class Connecting implements MyState {
  const Connecting();
  






@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Connecting);
}


@override
int get hashCode => runtimeType.hashCode;

@override
String toString() {
  return 'MyState.connecting()';
}


}




/// @nodoc


class Connected implements MyState {
  const Connected(this.connectedState);
  

 final  ConnectedState connectedState;

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@JsonKey(includeFromJson: false, includeToJson: false)
@pragma('vm:prefer-inline')
$ConnectedCopyWith<Connected> get copyWith => _$ConnectedCopyWithImpl<Connected>(this, _$identity);



@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Connected&&(identical(other.connectedState, connectedState) || other.connectedState == connectedState));
}


@override
int get hashCode => Object.hash(runtimeType,connectedState);

@override
String toString() {
  return 'MyState.connected(connectedState: $connectedState)';
}


}

/// @nodoc
abstract mixin class $ConnectedCopyWith<$Res> implements $MyStateCopyWith<$Res> {
  factory $ConnectedCopyWith(Connected value, $Res Function(Connected) _then) = _$ConnectedCopyWithImpl;
@useResult
$Res call({
 ConnectedState connectedState
});


$ConnectedStateCopyWith<$Res> get connectedState;

}
/// @nodoc
class _$ConnectedCopyWithImpl<$Res>
    implements $ConnectedCopyWith<$Res> {
  _$ConnectedCopyWithImpl(this._self, this._then);

  final Connected _self;
  final $Res Function(Connected) _then;

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@pragma('vm:prefer-inline') $Res call({Object? connectedState = null,}) {
  return _then(Connected(
null == connectedState ? _self.connectedState : connectedState // ignore: cast_nullable_to_non_nullable
as ConnectedState,
  ));
}

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@override
@pragma('vm:prefer-inline')
$ConnectedStateCopyWith<$Res> get connectedState {
  
  return $ConnectedStateCopyWith<$Res>(_self.connectedState, (value) {
    return _then(_self.copyWith(connectedState: value));
  });
}
}

/// @nodoc


class Error implements MyState {
  const Error(this.errorMessage);
  

 final  String errorMessage;

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@JsonKey(includeFromJson: false, includeToJson: false)
@pragma('vm:prefer-inline')
$ErrorCopyWith<Error> get copyWith => _$ErrorCopyWithImpl<Error>(this, _$identity);



@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Error&&(identical(other.errorMessage, errorMessage) || other.errorMessage == errorMessage));
}


@override
int get hashCode => Object.hash(runtimeType,errorMessage);

@override
String toString() {
  return 'MyState.error(errorMessage: $errorMessage)';
}


}

/// @nodoc
abstract mixin class $ErrorCopyWith<$Res> implements $MyStateCopyWith<$Res> {
  factory $ErrorCopyWith(Error value, $Res Function(Error) _then) = _$ErrorCopyWithImpl;
@useResult
$Res call({
 String errorMessage
});




}
/// @nodoc
class _$ErrorCopyWithImpl<$Res>
    implements $ErrorCopyWith<$Res> {
  _$ErrorCopyWithImpl(this._self, this._then);

  final Error _self;
  final $Res Function(Error) _then;

/// Create a copy of MyState
/// with the given fields replaced by the non-null parameter values.
@pragma('vm:prefer-inline') $Res call({Object? errorMessage = null,}) {
  return _then(Error(
null == errorMessage ? _self.errorMessage : errorMessage // ignore: cast_nullable_to_non_nullable
as String,
  ));
}


}

/// @nodoc
mixin _$ConnectedState {

 Color get color;
/// Create a copy of ConnectedState
/// with the given fields replaced by the non-null parameter values.
@JsonKey(includeFromJson: false, includeToJson: false)
@pragma('vm:prefer-inline')
$ConnectedStateCopyWith<ConnectedState> get copyWith => _$ConnectedStateCopyWithImpl<ConnectedState>(this as ConnectedState, _$identity);



@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is ConnectedState&&(identical(other.color, color) || other.color == color));
}


@override
int get hashCode => Object.hash(runtimeType,color);

@override
String toString() {
  return 'ConnectedState(color: $color)';
}


}

/// @nodoc
abstract mixin class $ConnectedStateCopyWith<$Res>  {
  factory $ConnectedStateCopyWith(ConnectedState value, $Res Function(ConnectedState) _then) = _$ConnectedStateCopyWithImpl;
@useResult
$Res call({
 Color color
});




}
/// @nodoc
class _$ConnectedStateCopyWithImpl<$Res>
    implements $ConnectedStateCopyWith<$Res> {
  _$ConnectedStateCopyWithImpl(this._self, this._then);

  final ConnectedState _self;
  final $Res Function(ConnectedState) _then;

/// Create a copy of ConnectedState
/// with the given fields replaced by the non-null parameter values.
@pragma('vm:prefer-inline') @override $Res call({Object? color = null,}) {
  return _then(ConnectedState(
color: null == color ? _self.color : color // ignore: cast_nullable_to_non_nullable
as Color,
  ));
}

}


/// Adds pattern-matching-related methods to [ConnectedState].
extension ConnectedStatePatterns on ConnectedState {
/// A variant of `map` that fallback to returning `orElse`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeMap<TResult extends Object?>({required TResult orElse(),}){
final _that = this;
switch (_that) {
case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// Callbacks receives the raw object, upcasted.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case final Subclass2 value:
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult map<TResult extends Object?>(){
final _that = this;
switch (_that) {
case _:
  throw StateError('Unexpected subclass');

}
}
/// A variant of `map` that fallback to returning `null`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? mapOrNull<TResult extends Object?>(){
final _that = this;
switch (_that) {
case _:
  return null;

}
}
/// A variant of `when` that fallback to an `orElse` callback.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeWhen<TResult extends Object?>({required TResult orElse(),}) {final _that = this;
switch (_that) {
case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// As opposed to `map`, this offers destructuring.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case Subclass2(:final field2):
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult when<TResult extends Object?>() {final _that = this;
switch (_that) {
case _:
  throw StateError('Unexpected subclass');

}
}
/// A variant of `when` that fallback to returning `null`
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? whenOrNull<TResult extends Object?>() {final _that = this;
switch (_that) {
case _:
  return null;

}
}

}

/// @nodoc
mixin _$Color {

 int get r; int get g; int get b;
/// Create a copy of Color
/// with the given fields replaced by the non-null parameter values.
@JsonKey(includeFromJson: false, includeToJson: false)
@pragma('vm:prefer-inline')
$ColorCopyWith<Color> get copyWith => _$ColorCopyWithImpl<Color>(this as Color, _$identity);



@override
bool operator ==(Object other) {
  return identical(this, other) || (other.runtimeType == runtimeType&&other is Color&&(identical(other.r, r) || other.r == r)&&(identical(other.g, g) || other.g == g)&&(identical(other.b, b) || other.b == b));
}


@override
int get hashCode => Object.hash(runtimeType,r,g,b);

@override
String toString() {
  return 'Color(r: $r, g: $g, b: $b)';
}


}

/// @nodoc
abstract mixin class $ColorCopyWith<$Res>  {
  factory $ColorCopyWith(Color value, $Res Function(Color) _then) = _$ColorCopyWithImpl;
@useResult
$Res call({
 int r, int g, int b
});




}
/// @nodoc
class _$ColorCopyWithImpl<$Res>
    implements $ColorCopyWith<$Res> {
  _$ColorCopyWithImpl(this._self, this._then);

  final Color _self;
  final $Res Function(Color) _then;

/// Create a copy of Color
/// with the given fields replaced by the non-null parameter values.
@pragma('vm:prefer-inline') @override $Res call({Object? r = null,Object? g = null,Object? b = null,}) {
  return _then(Color(
r: null == r ? _self.r : r // ignore: cast_nullable_to_non_nullable
as int,g: null == g ? _self.g : g // ignore: cast_nullable_to_non_nullable
as int,b: null == b ? _self.b : b // ignore: cast_nullable_to_non_nullable
as int,
  ));
}

}


/// Adds pattern-matching-related methods to [Color].
extension ColorPatterns on Color {
/// A variant of `map` that fallback to returning `orElse`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeMap<TResult extends Object?>({required TResult orElse(),}){
final _that = this;
switch (_that) {
case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// Callbacks receives the raw object, upcasted.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case final Subclass2 value:
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult map<TResult extends Object?>(){
final _that = this;
switch (_that) {
case _:
  throw StateError('Unexpected subclass');

}
}
/// A variant of `map` that fallback to returning `null`.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case final Subclass value:
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? mapOrNull<TResult extends Object?>(){
final _that = this;
switch (_that) {
case _:
  return null;

}
}
/// A variant of `when` that fallback to an `orElse` callback.
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return orElse();
/// }
/// ```

@optionalTypeArgs TResult maybeWhen<TResult extends Object?>({required TResult orElse(),}) {final _that = this;
switch (_that) {
case _:
  return orElse();

}
}
/// A `switch`-like method, using callbacks.
///
/// As opposed to `map`, this offers destructuring.
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case Subclass2(:final field2):
///     return ...;
/// }
/// ```

@optionalTypeArgs TResult when<TResult extends Object?>() {final _that = this;
switch (_that) {
case _:
  throw StateError('Unexpected subclass');

}
}
/// A variant of `when` that fallback to returning `null`
///
/// It is equivalent to doing:
/// ```dart
/// switch (sealedClass) {
///   case Subclass(:final field):
///     return ...;
///   case _:
///     return null;
/// }
/// ```

@optionalTypeArgs TResult? whenOrNull<TResult extends Object?>() {final _that = this;
switch (_that) {
case _:
  return null;

}
}

}

// dart format on
