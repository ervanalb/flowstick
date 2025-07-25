import 'package:bloc/bloc.dart';
import 'package:freezed_annotation/freezed_annotation.dart';
import 'dart:async';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter/foundation.dart' show kIsWeb;
import 'dart:io';

part 'state.freezed.dart';

@freezed
sealed class MyState with _$MyState {
  const factory MyState.initializing() = Initializing;
  const factory MyState.scanning(List<ScanResult> scanResults) = Scanning;
  const factory MyState.connecting() = Connecting;
  const factory MyState.connected(ConnectedState connectedState) = Connected;
  const factory MyState.error(String errorMessage) = Error;
}

@freezed
class ConnectedState with _$ConnectedState {
  ConnectedState({
    required this.color,
  });

  final Color color;
}

@freezed
class Color with _$Color {
  Color({
    required this.r,
    required this.g,
    required this.b,
  });

  final int r;
  final int g;
  final int b;
}

class MyCubit extends Cubit<MyState> {
  StreamSubscription<List<ScanResult>>? _scanResultsSubscription = null;

  MyCubit() : super(const MyState.initializing()) {
    _enterInitialize();
  }

  initialize() async {
    switch (state) {
      case Initializing():
        // Don't double up
        return;
      case Scanning(:final scanResults):
        _stopScanning();
      case Connecting():
        // TODO what to do here?
        throw Exception('Initializing while connecting is not implemented');
        return;
      case Connected():
      case Error(:final errorMessage):
    }
    return await _enterInitialize();
  }

  _enterInitialize() async {
    emit(MyState.initializing());

    // first, check if bluetooth is supported by your hardware
    // Note: The platform is initialized on the first call to any FlutterBluePlus method.
    try {
      if (await FlutterBluePlus.isSupported == false) {
          emit(MyState.error('Bluetooth is not supported on this hardware'));
          return;
      }
    } catch (e) {
      emit(MyState.error('Error initializing bluetooth: $e'));
      return;
    }

    // handle bluetooth on & off
    // note: for iOS the initial state is typically BluetoothAdapterState.unknown
    // note: if you have permissions issues you will get stuck at BluetoothAdapterState.unauthorized
    final Completer<void> completer = Completer<void>();
    var subscription = FlutterBluePlus.adapterState.listen(
      (state) {
        if (state == BluetoothAdapterState.on) {
          completer.complete();
        } else {
          completer.completeError(
            Exception('Bluetooth adapter state changed to: $state')
          );
        }
    },
    onError:
      (error) {
        // Handle stream errors
        if (!completer.isCompleted) {
          completer.completeError(error);
        }
      }
    );

    // turn on bluetooth ourself if we can
    // for iOS, the user controls bluetooth enable/disable
    if (!kIsWeb && Platform.isAndroid) {
      FlutterBluePlus.turnOn(); // don't await
    }

    var a = FlutterBluePlus.adapterStateNow;

    try {
      await completer.future;
    } catch (e) {
      emit(MyState.error('$e'));
      return;
    } finally {
      await subscription.cancel();
    }

    // Ok!
    _startScanning();
  }

  _startScanning() {
    _stopScanning(); // Should never be required, but we'll do it for safety

    _scanResultsSubscription = FlutterBluePlus.onScanResults.listen(
      (results) {
        emit(MyState.scanning(results));
      },
      onError: (e) {
        _stopScanning();
        emit(MyState.error('$e'));
      },
    );

    FlutterBluePlus.startScan(
      withServices:[Guid.fromBytes([0x18,0x09])],
    );

    emit(MyState.scanning([]));
  }

  _stopScanning() {
    if (_scanResultsSubscription != null) { 
      FlutterBluePlus.stopScan();
      _scanResultsSubscription?.cancel();
      _scanResultsSubscription = null;
    }
  }
}
